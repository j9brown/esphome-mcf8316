# esphome-mcf8316

An ESPHome component for driving the [MCF8316D](https://www.ti.com/product/MCF8316D) sensorless field-oriented control brushless DC motor controller with an I2C interface.

This component is not compatible with the MCF8316A or MCF8316C revisions which have different register layouts (and [errata](https://e2e.ti.com/support/motor-drivers-group/motor-drivers/f/motor-drivers-forum/1066777/faq-production-device-information)).

**Status: WORK IN PROGRESS**

## Component schema

```yaml
mcf8316:
    - id: minuet_fan_motor_driver
        wake: <SPEED/WAKE PIN>
        nfault: <NFAULT PIN>
        watchdog: true
        on_fault:
            then:
                # Handle FaultStatus in 'x' object
```

* on_fault: triggered when a fault is reported, changes, or is cleared after being reported

## Resources

- [MCF8316D datasheet](https://www.ti.com/lit/ds/symlink/mcf8316d.pdf)
- [Tuning guide](https://www.ti.com/lit/an/slla663/slla663.pdf)
