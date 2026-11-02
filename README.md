# Self-Balancing Platform (Thrust Vector Control Model)

A two-axis platform that holds itself level against disturbances. It models how thrust vector control keeps a rocket pointed true.

Course: ME 4990L Mechatronics Laboratory, Cal Poly Pomona, Fall 2024. Team of 3.

## What it does

- Reads tilt from an MPU6050 accelerometer and gyroscope over I2C.
- Estimates roll and pitch as a fixed-weight blend of the integrated gyroscope angle (96%) and the accelerometer angle (4%). The gyroscope signal is first bias-corrected from stationary readings and smoothed with a low-pass filter (alpha = 0.9).
- Corrects tilt with a PID controller driving two servos on an Arduino Uno.
- Result in testing: angle error held within 2 degrees.

## Hardware

- Arduino Uno
- MPU6050 IMU
- Two hobby servos (pins 9 and 10)
- 3D-printed pivoting frame designed in CAD

## Files

- `self_balancing_platform.ino`: the Arduino sketch, as printed in the report appendix and reformatted only.
- The full report with CAD, wiring and results is on the [portfolio](https://esperiquette.github.io/portfolio/).

## Gains used

Kp = 0.5, Ki = 0.01, Kd = 0.1, tuned by repeated tests.

## Known limitations

- The orientation estimate is not a complementary filter in the strict sense. The gyroscope angle is integrated on its own and is never corrected by the accelerometer, so gyroscope drift accumulates. The 96/4 weights scale the two angles but do not remove the drift. A complementary filter feeds the fused estimate back into the integration, for example `angle = 0.96 * (angle + gyro * dt) + 0.04 * accel_angle`, which keeps the drift bounded.
- `calculate_IMU_error()` is a placeholder. The accelerometer and gyroscope offsets are zero, so only the stationary gyroscope bias averaging in `calculateGyroBias()` is applied.
- The first loop iteration computes its elapsed time from zero, which can briefly give a large derivative term.
- A stepper-driven propeller to add a thrust disturbance was designed but not integrated in time.

## Next steps

Feed the fused estimate back into the angle integration, then try a Kalman filter. Further work: a third degree of freedom, stepper motors for smoother actuation, and a microcontroller with more I/O.

## Credits

Course project completed by a team of three.
