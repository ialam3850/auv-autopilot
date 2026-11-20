# Fault behavior

Invalid sensor values produce neutral actuator messages without changing
the last healthy filter state. Valid samples can resume operation. An invalid
enabled command disables control. Bounds are conservative software guards,
not a certified vehicle operating envelope.

Hardware validation and physical actuator shutdown are pending.

HIL disables control and sends a neutral reply after 100 ms without an
accepted sensor timestamp. A fresh enabled command is needed to re-arm.
The bridge sends commands at setpoint changes and once per second.
This firmware sends serial actuator messages; physical PWM outputs are not
implemented, so this is not a physical motor cutoff.
