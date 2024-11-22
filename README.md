# FlyDuino

This is an experimental flight controller for a quadcopter. It implements the following features:

- PID controller for pitch, yaw and roll
- Bluetooth communication to configure the PID gains
- Orientation is derived from the onboard IMU. Using both acceleration and angular rate to determine a stable 3D orientation.
- RC control, using FlySky ppm protocol to decode throttle position and user pitch/raw/roll
- Debugging using [processing](https://processing.org/)
