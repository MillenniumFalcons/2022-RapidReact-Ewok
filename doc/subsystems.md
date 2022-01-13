# Subsystems

| Subsystem   | # motors | # solenoids | Extra Sensors | Closed loop | Output          | SysId                         | Notes                     |
|-------------|----------|-------------|---------------|-------------|-----------------|-------------------------------|---------------------------|
| Drivetrain  | 4        | 0           | Gyro          | Velocity    | Robot speed     | Yes                           |                           |
| Intake      | 1        | 1           | N/A           | Velocity    | Surface speed   | possible                      |                           |
| Sorter      | 1        | 0           | Color sensor  | Velocity    | Surface speed   | possible                      |                           |
| Feeder      | 2        | 0           | Beam break    | "Pos,vel"   | probably inches | With and without cargo        |                           |
| BallStopper | 0        | 1           | N/A           | N/A         | bool            | N/A                           |                           |
| Turret      | 1        | 0           | N/A           | Position    | Angle (degrees) | static friction               |                           |
| Shooter     | 2        | 0           | N/A           | Velocity    | RPM             | Yes                           |                           |
| Climber     | 1        | 2           | limit switch? | Position    | feet            | "Yes, with and without robot" | 2x                        |
| Vision      | 0        | 0           | 1-3 cameras   | ..          | degrees         | ..                            | plugged to PDP like motor |

