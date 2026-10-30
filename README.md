# Self-Balancing Platform (Thrust Vector Control Model)

A two-axis platform that holds itself level against disturbances. It models how thrust vector control keeps a rocket pointed true.

Course: ME 4990L Mechatronics Laboratory, Cal Poly Pomona, Fall 2024. Team of 3.

## What it does

- Reads tilt from an MPU6050 accelerometer and gyroscope over I2C.
- Combines the two with a complementary filter (96% gyroscope, 4% accelerometer).
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

- `calculate_IMU_error()` is a placeholder. The accelerometer and gyroscope offsets are zero, so only the stationary gyroscope bias averaging in `calculateGyroBias()` is applied.
- The gyroscope angle is integrated separately and is not corrected by the fused output, so gyroscope drift is only reduced by the 4% accelerometer weight each cycle.
- The first loop iteration computes its elapsed time from zero, which can briefly give a large derivative term.
- A stepper-driven propeller to add a thrust disturbance was designed but not integrated in time.

## Next steps

Kalman filter, a third degree of freedom, stepper motors for smoother actuation, and a microcontroller with more I/O.

## Credits

Luke Esperiquette, Sairam Sreekant, James Willis
