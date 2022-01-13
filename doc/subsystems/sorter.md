# Sorter

This subsystem is responsible for spitting out balls of the wrong color.


## Motors
- 1 Falcon 500 motor which can invert direction to spit out balls

## Solenoids
- 1 solenoid that controls 2 pistons which extends the intake
  

## Sensors
- Falcon built in encoders
- REV Color sensor


## Closed loop
Using built in Falcon 500 encoders.

For closed loop we want to run at the same surface speed of the intake, both when we want to spit out, and for loading in balls.

### Units
- Position: N/A
- Velocity: m/s
- Acceleration: m/s^2


## SysId
- Need to find kS, kV (kA probably doesn't matter)

