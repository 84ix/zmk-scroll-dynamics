# zmk-scroll-dynamics

ZMK input processor for trackball scroll mode. It combines raw pointer-to-scroll
mapping, zero-delay axis snap, acceleration, inertia, and fixed-point remainder
quantization into one processor.

See `zmk-scroll-input-plan.html` for the design notes.

## Example

```dts
/ {
    scroll_dynamics: scroll_dynamics {
        compatible = "zmk,input-processor-scroll-dynamics";
        #input-processor-cells = <0>;

        axis-mode = "snap";
        output-axis = "auto";

        input-scale = <1000>;
        wheel-step = <75>;
        output-multiplier = <1>;
        output-divisor = <1>;

        snap-ratio = <1400>;
        snap-switch-ratio = <1800>;
        snap-idle-ms = <80>;
        minor-axis-scale = <0>;

        min-factor = <800>;
        max-factor = <2400>;
        speed-threshold = <900>;
        speed-max = <4500>;
        acceleration-exponent = <1>;
        track-remainders;

        inertia-start-speed = <1300>;
        inertia-start-distance = <8>;
        inertia-min-events = <3>;
        inertia-decay = <860>;
        inertia-stop-speed = <80>;
        inertia-tick-ms = <8>;
        reverse-cancel;
    };
};
```

```dts
scrollball_listener {
    compatible = "zmk,input-listener";
    device = <&scrollball>;
    input-processors =
        <&zip_xy_transform (INPUT_TRANSFORM_XY_SWAP|INPUT_TRANSFORM_X_INVERT)>,
        <&scroll_dynamics>;
};
```
