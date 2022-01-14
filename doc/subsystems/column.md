# Column

This subsystem is responsible for moving balls from the bottom feeder to the shooter.


## Motors
- 1 Falcon 500 motor
  

## Sensors
- Falcon built in encoders
- 1 Beam break sensor at the top of the column


## Closed loop
Using built in Falcon 500 encoders.

For closed loop we want to be able to move a ball a set amount of distance in the column, but we also want to be able to move in a specific velocity for when we want to match the speed of the shooter and (kicker wheels?).

We want the surface speed of the belts that are touching the balls.

## Units
- Position: meters
- Velocity: m/s
- Acceleration: m/s^2


## SysId
Here we want to find all the characterization constants both with and without balls.

- Need to find kS, kV (kA probably doesn't matter)

