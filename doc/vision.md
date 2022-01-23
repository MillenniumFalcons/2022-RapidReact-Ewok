# Vision Targeting Pipeline
## RobotTracker.java
- Keep track of robot pose over time
- Keep track of turret rotation over time
- Latest Twist2d (this is for adjusting the turret velocity by the velocity of the robot)

## MultiTargetTracker.java
- keep track of multiple targets over time

## TrackedTarget.java
- keep track of a single target over time

## TargetInfo.java
- A single frame of a target that passed the limelight pipeline



## Big picture view
### Keep track of targets
1. `RobotTracker` keeps track of robot pose on the field over time
2. `RobotTracker` keeps track of turret rotation over time
3. `VisionController` sees target: Yaw, Pitch, Latency
4. ?? converts target to camToTarget transform
5. ?? converts camToTarget to fieldToTarget (fieldToRobot <- robotToTurret <- turretToCam <- camToTarget)
6. ?? updates `MultiTargetTracker` using fieldToTarget Pose2d
7. `MultiTargetTracker` compares new observation to old observation
8. `MultiTargetTracker` either adds to old observation, or creates new one depending on the real distance between new Observerd and old
9. `MultiTargetTracker` calculates the new average position for the updated `TrackedTarget`

### Try to aim
1. `RobotTracker` keeps track of robot pose on the field over time
2. `RobotTracker` keeps track of turret rotation over time
3. `AimTurret` command wants the latest position and FF to set to the turret from ??
4. `AccelerateFlywheel` command wants distance to target from ??
5. `AimHood` command wants distance to target from ??
6. ?? asks `MultiTargetTracker` for all tracked targets, A a list of `TrackedTarget`
7. ?? sorts the `List<TrackedTarget>` using the `TrackedTargetComparator`
8. ?? gets the first element of the list and constructs a new `AimingParameters` object
8. ?? asks `RobotTracker` to convert fieldToTarget to turretToTarget ( inverse(robotToTurret) <- inverse(fieldToRobot) <- fieldToTarget)
10. ?? gets the drivetrain speed `Twist2d` (dx, dy=0, dtheta) from `RobotTracker`
11. ?? gets the turretToTarget(timestamp) from `RobotTracker`
12. ?? uses turretToTarget rotation2d component for calculating turret position error
13. ?? uses drivetrain speed to calculate turret FF
14. ?? returns values to the `AimTurret`
15. ?? uses turretToTarget norm of translation2d to get distance
16. ?? returns values to `AccelerateFlywheel` and `AimHood`