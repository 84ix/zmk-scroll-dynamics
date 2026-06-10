/*
 * ZMK Input Processor: Scroll Dynamics
 *
 * Combines raw pointer-to-scroll mapping, zero-delay axis lock,
 * acceleration, inertia, and fixed-point wheel quantization.
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_scroll_dynamics

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <stdint.h>

#include <drivers/input_processor.h>
#include <zmk/endpoints.h>
#include <zmk/hid.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if IS_ENABLED(CONFIG_ZMK_SPLIT) && !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#error "zmk,input-processor-scroll-dynamics must run on the central side because inertia emits ZMK mouse HID reports"
#endif

#define AXIS_MODE_X 0
#define AXIS_MODE_Y 1
#define AXIS_MODE_SNAP 2

#define OUTPUT_AXIS_WHEEL 0
#define OUTPUT_AXIS_HWHEEL 1
#define OUTPUT_AXIS_AUTO 2

enum locked_axis {
    AXIS_NONE,
    AXIS_X,
    AXIS_Y,
};

struct scroll_dynamics_config {
    uint8_t axis_mode;
    uint8_t output_axis;
    bool invert_x;
    bool invert_y;
    int32_t input_scale;
    int32_t wheel_step;
    int32_t output_divisor;
    int32_t min_factor;
    int32_t max_factor;
    int32_t speed_threshold;
    int32_t speed_max;
    int32_t acceleration_exponent;
    int32_t snap_ratio;
    int32_t snap_switch_ratio;
    int32_t snap_idle_ms;
    int32_t minor_axis_scale;
    int32_t inertia_start_speed;
    int32_t inertia_start_distance;
    uint8_t inertia_min_events;
    int32_t inertia_decay;
    int32_t inertia_stop_speed;
    int32_t inertia_tick_ms;
    bool reverse_cancel;
    bool track_remainders;
};

struct scroll_dynamics_data {
    const struct device *dev;
    enum locked_axis locked_axis;
    int64_t last_event_ms;
    int64_t last_input_ms;
    int32_t remainder_x;
    int32_t remainder_y;
    int32_t output_remainder_x;
    int32_t output_remainder_y;
    int32_t pending_x;
    int32_t pending_y;
    int32_t inertia_velocity;
    int32_t gesture_distance;
    uint8_t gesture_events;
    int8_t last_direction;
    bool inertia_active;
    struct k_work_delayable inertia_work;
};

static int32_t abs32(int32_t v) { return v < 0 ? -v : v; }

static int8_t sign32(int32_t v) {
    if (v > 0) {
        return 1;
    }
    if (v < 0) {
        return -1;
    }
    return 0;
}

static void cancel_inertia(struct scroll_dynamics_data *data) {
    data->inertia_active = false;
    data->inertia_velocity = 0;
    k_work_cancel_delayable(&data->inertia_work);
}

static void reset_gesture(struct scroll_dynamics_data *data) {
    data->locked_axis = AXIS_NONE;
    data->gesture_distance = 0;
    data->gesture_events = 0;
    data->last_direction = 0;
}

static int32_t safe_divisor(int32_t value, int32_t fallback) {
    return value <= 0 ? fallback : value;
}

static enum locked_axis configured_axis(const struct scroll_dynamics_config *cfg) {
    switch (cfg->axis_mode) {
    case AXIS_MODE_X:
        return AXIS_X;
    case AXIS_MODE_Y:
        return AXIS_Y;
    default:
        return AXIS_NONE;
    }
}

static enum locked_axis choose_axis(const struct scroll_dynamics_config *cfg,
                                    struct scroll_dynamics_data *data, int32_t dx,
                                    int32_t dy) {
    enum locked_axis fixed = configured_axis(cfg);
    if (fixed != AXIS_NONE) {
        data->locked_axis = fixed;
        return fixed;
    }

    int32_t abs_x = abs32(dx);
    int32_t abs_y = abs32(dy);

    if (data->locked_axis == AXIS_NONE) {
        if ((int64_t)abs_x * 1000 >= (int64_t)abs_y * cfg->snap_ratio) {
            data->locked_axis = AXIS_X;
        } else if ((int64_t)abs_y * 1000 >= (int64_t)abs_x * cfg->snap_ratio) {
            data->locked_axis = AXIS_Y;
        } else {
            data->locked_axis = abs_x >= abs_y ? AXIS_X : AXIS_Y;
        }
    } else if (data->locked_axis == AXIS_X &&
               (int64_t)abs_y * 1000 > (int64_t)abs_x * cfg->snap_switch_ratio) {
        data->locked_axis = AXIS_Y;
        data->remainder_x = 0;
        data->output_remainder_x = 0;
        cancel_inertia(data);
    } else if (data->locked_axis == AXIS_Y &&
               (int64_t)abs_x * 1000 > (int64_t)abs_y * cfg->snap_switch_ratio) {
        data->locked_axis = AXIS_X;
        data->remainder_y = 0;
        data->output_remainder_y = 0;
        cancel_inertia(data);
    }

    return data->locked_axis;
}

static int32_t acceleration_factor(const struct scroll_dynamics_config *cfg, int32_t speed) {
    if (speed <= cfg->speed_threshold) {
        return cfg->min_factor;
    }

    int32_t range = cfg->speed_max - cfg->speed_threshold;
    if (range <= 0) {
        return cfg->max_factor;
    }

    int32_t t = CLAMP((int64_t)(speed - cfg->speed_threshold) * 1000 / range, 0, 1000);
    if (cfg->acceleration_exponent == 2) {
        t = (int64_t)t * t / 1000;
    }

    return cfg->min_factor + (int64_t)(cfg->max_factor - cfg->min_factor) * t / 1000;
}

static int32_t scale_delta(const struct scroll_dynamics_config *cfg, int32_t delta, int32_t dt_ms) {
    int32_t speed = (int64_t)abs32(delta) * 1000 / MAX(dt_ms, 1);
    int32_t factor = acceleration_factor(cfg, speed);

    return (int64_t)delta * cfg->input_scale * factor / 1000000;
}

static int16_t quantize(const struct scroll_dynamics_config *cfg, int32_t delta, int32_t *remainder) {
    int32_t step = safe_divisor(cfg->wheel_step, 1000);
    int32_t acc = cfg->track_remainders ? *remainder + delta : delta;
    int32_t units = acc / step;

    if (cfg->track_remainders) {
        *remainder = acc - units * step;
    }

    return CLAMP(units, INT16_MIN, INT16_MAX);
}

static void suppress_pointer_event(struct input_event *event) {
    event->type = INPUT_EV_REL;
    event->code = INPUT_REL_WHEEL;
    event->value = 0;
}

static uint16_t output_code(const struct scroll_dynamics_config *cfg, enum locked_axis axis) {
    if (cfg->output_axis == OUTPUT_AXIS_WHEEL) {
        return INPUT_REL_WHEEL;
    }
    if (cfg->output_axis == OUTPUT_AXIS_HWHEEL) {
        return INPUT_REL_HWHEEL;
    }
    return axis == AXIS_X ? INPUT_REL_HWHEEL : INPUT_REL_WHEEL;
}

static int32_t active_remainder_delta(struct scroll_dynamics_data *data, enum locked_axis axis) {
    return axis == AXIS_X ? data->remainder_x : data->remainder_y;
}

static void set_active_remainder(struct scroll_dynamics_data *data, enum locked_axis axis,
                                 int32_t value) {
    if (axis == AXIS_X) {
        data->remainder_x = value;
    } else {
        data->remainder_y = value;
    }
}

static int32_t active_output_remainder(struct scroll_dynamics_data *data, enum locked_axis axis) {
    return axis == AXIS_X ? data->output_remainder_x : data->output_remainder_y;
}

static void set_active_output_remainder(struct scroll_dynamics_data *data, enum locked_axis axis,
                                        int32_t value) {
    if (axis == AXIS_X) {
        data->output_remainder_x = value;
    } else {
        data->output_remainder_y = value;
    }
}

static int16_t emit_units_for_axis(const struct scroll_dynamics_config *cfg,
                                   struct scroll_dynamics_data *data, enum locked_axis axis,
                                   int32_t delta) {
    int32_t remainder = active_remainder_delta(data, axis);
    int16_t units = quantize(cfg, delta, &remainder);
    set_active_remainder(data, axis, remainder);

    int32_t divisor = safe_divisor(cfg->output_divisor, 1);
    if (divisor <= 1 || units == 0) {
        return units;
    }

    int32_t output_remainder = active_output_remainder(data, axis);
    int32_t acc = output_remainder + units;
    int16_t output_units = CLAMP(acc / divisor, INT16_MIN, INT16_MAX);
    output_remainder = acc - output_units * divisor;
    set_active_output_remainder(data, axis, output_remainder);

    return output_units;
}

static void send_mouse_scroll(int16_t hwheel, int16_t wheel) {
    if (hwheel == 0 && wheel == 0) {
        return;
    }

    zmk_hid_mouse_movement_set(0, 0);
    zmk_hid_mouse_scroll_set(hwheel, wheel);
#ifdef ZMK_ENDPOINT_NONE_COUNT
    zmk_endpoint_send_mouse_report();
#else
    zmk_endpoints_send_mouse_report();
#endif
    zmk_hid_mouse_scroll_set(0, 0);
}

static void inertia_tick(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct scroll_dynamics_data *data = CONTAINER_OF(dwork, struct scroll_dynamics_data, inertia_work);
    const struct scroll_dynamics_config *cfg = data->dev->config;

    if (!data->inertia_active || data->locked_axis == AXIS_NONE) {
        return;
    }

    int32_t tick_ms = safe_divisor(cfg->inertia_tick_ms, 8);
    int32_t delta = (int64_t)data->inertia_velocity * tick_ms / 1000;
    int16_t units = emit_units_for_axis(cfg, data, data->locked_axis, delta);

    if (units != 0) {
        if (output_code(cfg, data->locked_axis) == INPUT_REL_HWHEEL) {
            send_mouse_scroll(units, 0);
        } else {
            send_mouse_scroll(0, units);
        }
    }

    data->inertia_velocity = (int64_t)data->inertia_velocity * cfg->inertia_decay / 1000;
    if (abs32(data->inertia_velocity) < cfg->inertia_stop_speed) {
        cancel_inertia(data);
        return;
    }

    k_work_schedule(&data->inertia_work, K_MSEC(tick_ms));
}

static int scroll_dynamics_init(const struct device *dev) {
    struct scroll_dynamics_data *data = dev->data;
    data->dev = dev;
    k_work_init_delayable(&data->inertia_work, inertia_tick);
    return 0;
}

static int scroll_dynamics_handle_event(const struct device *dev, struct input_event *event,
                                        uint32_t param1, uint32_t param2,
                                        struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    if (event->type != INPUT_EV_REL ||
        (event->code != INPUT_REL_X && event->code != INPUT_REL_Y)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    const struct scroll_dynamics_config *cfg = dev->config;
    struct scroll_dynamics_data *data = dev->data;
    int64_t now = k_uptime_get();
    int32_t dt_ms = data->last_event_ms == 0 ? 1 : MAX(now - data->last_event_ms, 1);

    if (data->last_input_ms == 0 || now - data->last_input_ms > cfg->snap_idle_ms) {
        reset_gesture(data);
    }

    if (event->code == INPUT_REL_X) {
        data->pending_x += event->value;
    } else {
        data->pending_y += event->value;
    }

    if (!event->sync) {
        suppress_pointer_event(event);
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int32_t dx = data->pending_x;
    int32_t dy = data->pending_y;
    data->pending_x = 0;
    data->pending_y = 0;

    if (cfg->invert_x) {
        dx = -dx;
    }
    if (cfg->invert_y) {
        dy = -dy;
    }

    enum locked_axis axis = choose_axis(cfg, data, dx, dy);
    int32_t raw_delta = axis == AXIS_X ? dx : dy;
    int32_t minor_delta = axis == AXIS_X ? dy : dx;
    raw_delta += (int64_t)minor_delta * cfg->minor_axis_scale / 1000;

    if (raw_delta == 0) {
        data->last_event_ms = now;
        data->last_input_ms = now;
        suppress_pointer_event(event);
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int32_t scaled_delta = scale_delta(cfg, raw_delta, dt_ms);
    int8_t direction = sign32(scaled_delta);

    if (cfg->reverse_cancel && direction != 0 && data->last_direction != 0 &&
        direction != data->last_direction) {
        cancel_inertia(data);
        data->gesture_distance = 0;
        data->gesture_events = 0;
    }

    data->gesture_distance += abs32(scaled_delta);
    if (data->gesture_events < UINT8_MAX) {
        data->gesture_events++;
    }
    data->last_direction = direction;

    int32_t speed = (int64_t)abs32(raw_delta) * 1000 / MAX(dt_ms, 1);
    if (speed >= cfg->inertia_start_speed &&
        data->gesture_distance >= cfg->inertia_start_distance &&
        data->gesture_events >= cfg->inertia_min_events) {
        data->inertia_velocity = (int64_t)scaled_delta * 1000 / MAX(dt_ms, 1);
        data->inertia_active = true;
        k_work_reschedule(&data->inertia_work, K_MSEC(safe_divisor(cfg->inertia_tick_ms, 8)));
    }

    int16_t units = emit_units_for_axis(cfg, data, axis, scaled_delta);

    data->last_event_ms = now;
    data->last_input_ms = now;

    if (units == 0) {
        event->code = output_code(cfg, axis);
        event->value = 0;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    event->code = output_code(cfg, axis);
    event->value = units;
    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api scroll_dynamics_driver_api = {
    .handle_event = scroll_dynamics_handle_event,
};

#define SCROLL_DYNAMICS_INST(n)                                                                   \
    BUILD_ASSERT(DT_INST_PROP(n, wheel_step) > 0, "wheel-step must be greater than 0");           \
    BUILD_ASSERT(DT_INST_PROP(n, output_divisor) > 0, "output-divisor must be greater than 0");   \
    BUILD_ASSERT(DT_INST_PROP(n, speed_max) > DT_INST_PROP(n, speed_threshold),                   \
                 "speed-max must be greater than speed-threshold");                              \
    static const struct scroll_dynamics_config scroll_dynamics_config_##n = {                     \
        .axis_mode = DT_ENUM_IDX(DT_DRV_INST(n), axis_mode),                                      \
        .output_axis = DT_ENUM_IDX(DT_DRV_INST(n), output_axis),                                  \
        .invert_x = DT_INST_PROP(n, invert_x),                                                    \
        .invert_y = DT_INST_PROP(n, invert_y),                                                    \
        .input_scale = DT_INST_PROP(n, input_scale),                                              \
        .wheel_step = DT_INST_PROP(n, wheel_step),                                                \
        .output_divisor = DT_INST_PROP(n, output_divisor),                                        \
        .min_factor = DT_INST_PROP(n, min_factor),                                                \
        .max_factor = DT_INST_PROP(n, max_factor),                                                \
        .speed_threshold = DT_INST_PROP(n, speed_threshold),                                      \
        .speed_max = DT_INST_PROP(n, speed_max),                                                  \
        .acceleration_exponent = DT_INST_PROP(n, acceleration_exponent),                          \
        .snap_ratio = DT_INST_PROP(n, snap_ratio),                                                \
        .snap_switch_ratio = DT_INST_PROP(n, snap_switch_ratio),                                  \
        .snap_idle_ms = DT_INST_PROP(n, snap_idle_ms),                                            \
        .minor_axis_scale = DT_INST_PROP(n, minor_axis_scale),                                    \
        .inertia_start_speed = DT_INST_PROP(n, inertia_start_speed),                              \
        .inertia_start_distance = DT_INST_PROP(n, inertia_start_distance),                        \
        .inertia_min_events = DT_INST_PROP(n, inertia_min_events),                                \
        .inertia_decay = DT_INST_PROP(n, inertia_decay),                                          \
        .inertia_stop_speed = DT_INST_PROP(n, inertia_stop_speed),                                \
        .inertia_tick_ms = DT_INST_PROP(n, inertia_tick_ms),                                      \
        .reverse_cancel = DT_INST_PROP(n, reverse_cancel),                                        \
        .track_remainders = DT_INST_PROP(n, track_remainders),                                    \
    };                                                                                            \
    static struct scroll_dynamics_data scroll_dynamics_data_##n;                                  \
    DEVICE_DT_INST_DEFINE(n, scroll_dynamics_init, NULL, &scroll_dynamics_data_##n,               \
                          &scroll_dynamics_config_##n, POST_KERNEL,                              \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &scroll_dynamics_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SCROLL_DYNAMICS_INST)
