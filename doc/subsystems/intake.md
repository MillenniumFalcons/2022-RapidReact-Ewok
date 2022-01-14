# Intake

This subsystem is responsible for getting balls from the floor to the bottom feeder.


## Motors
- 1 Falcon 500 motors which spins both intake rollers together

## Solenoids
- 1 solenoid that controls 2 pistons which extends the intake
  

## Sensors
- Falcon built in encoders


## Closed loop
Using built in Falcon 500 encoders.

For closed loop we want to run the rollers in 2x the speed of the drivetrain in order to intake balls when the drivetrain is driving towards them.

When the drivetrain is moving towards the balls the velocity of the drivetrain "cancels out" the movement of the roller, so they need to spin faster. This is why the units of the closed loop control are in m/s (surface speed of the rollers)

### Units
- Position: N/A
- Velocity: m/s
- Acceleration: m/s^2


## SysId
- Need to find kS, kV (kA probably doesn't matter)

