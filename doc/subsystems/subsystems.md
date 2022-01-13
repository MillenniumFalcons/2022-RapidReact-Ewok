# Subsystems

| Subsystem                          | # motors | # solenoids | Extra Sensors                             | Closed loop | Output (units)      | SysId                       | Notes                     |
|------------------------------------|----------|-------------|-------------------------------------------|-------------|---------------------|-----------------------------|---------------------------|
| [Drivetrain](./drivetrain.md)      | 4        | 0           | Gyro                                      | Velocity    | Robot speed   (m/s) | Yes                         |                           |
| [Intake](./intake.md)              | 1        | 1           | N/A                                       | Velocity    | Surface speed (m/s) | possible                    |                           |
| [Sorter](./sorter.md)              | 1        | 0           | Color sensor                              | Velocity    | Surface speed (m/s) | possible                    |                           |
| [Column](./column.md)              | 1        | 0           | Beam break                                | "Pos,vel"   | Surface speed (m/s) | With and without cargo      |                           |
| [Bottom Feeder](./bottom-feeder.md)| 1        | 0           | N/A                                       | "Pos,vel"   | Surface speed (m/s) | With and without cargo      |                           |
| [Ball Stopper](./ball-stopper.md)  | 0        | 1           | N/A                                       | N/A         | bool                | N/A                         |                           |
| [Kicker Wheels](./kicker-wheels.md)| 1        | 1           | N/A                                       | N/A         | Surface speed (m/s) | N/A                         |                           |
| [Turret](./turret.md)              | 1        | 0           | limit switch                              | Position    | Angle (degrees)     | static friction             |                           |
| [Shooter](./shooter.md)            | 2        | 0           | N/A                                       | Velocity    | m/s and RPM         | Yes                         |                           |
| [Hood](./hood.md)                  | 1        | 0           | limit switch, and encoder on output shaft | Position    | Angle (degrees)     | possible                    |                           |
| [Climber](./climber.md)            | 1        | 3           | limit switch?                             | Position    | Height (inches)     | Yes, with and without robot | 2x                        |
| [Vision](./vision.md)              | 1        | 0           | 1-3 cameras                               | ..          | degrees             | ..                          | plugged to PDP like motor |

