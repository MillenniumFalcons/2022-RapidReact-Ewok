# Drivetrain
This is the subsystem that controls the drivetrain. It will have 4 falcon 500 motors, 2 on each side.

The right side will be inverted (in the falcon configuration).

The drivetrain also has a Pigeon for getting the heading (used in autonomous)

## Motors
- 4 Falcon 500 motors

## Sensors
- Falcon built in encoders
- Pigeon IMU

## Closed loop
Using built in Falcon 500 encoders.

## Units
- Position: meters
- Velocity: m/s
- Acceleration: m/s^2
- Angle - Pigeon IMU

## SysId
Need to find kS, kV and kA.
