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
