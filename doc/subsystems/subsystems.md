# Subsystems

| Subsystem                          | # motors | # solenoids | Extra Sensors | Closed loop | Output          | SysId                         | Notes                     |
|----------------------------------- |----------|-------------|---------------|-------------|-----------------|-------------------------------|---------------------------|
| [Drivetrain](./drivetrain.md)      | 4        | 0           | Gyro          | Velocity    | Robot speed     | Yes                           |                           |
| [Intake](./intake.md)              | 1        | 1           | N/A           | Velocity    | Surface speed   | possible                      |                           |
| [Sorter](./sorter.md)              | 1        | 0           | Color sensor  | Velocity    | Surface speed   | possible                      |                           |
| [Column](./column.md)              | 1        | 0           | Beam break    | "Pos,vel"   | probably inches | With and without cargo        |                           |
| [Bottom Feeder](./bottom-feeder.md)| 1        | 0           | N/A           | "Pos,vel"   | probably inches | With and without cargo        |                           |
| [Ball Stopper](./ball-stopper.md)  | 0        | 1           | N/A           | N/A         | bool            | N/A                           |                           |
| [Turret](./turret.md)              | 1        | 0           | N/A           | Position    | Angle (degrees) | static friction               |                           |
| [Shooter](./shooter.md)            | 2        | 0           | N/A           | Velocity    | RPM             | Yes                           |                           |
| [Hood](./hood.md)                  | 1        | 0           | N/A           | Position    | Angle (degrees) | possible                      |                           |
| [Climber](./climber.md)            | 1        | 2           | limit switch? | Position    | feet            | "Yes, with and without robot" | 2x                        |
| Vision                             | 1        | 0           | 1-3 cameras   | ..          | degrees         | ..                            | plugged to PDP like motor |

