# Shooter

This subsystem is responsible for shooting balls from the column.


## Motors
- 2 Falcon 500 motors which are mechanically linked at the flywheel shaft (probably like 2020 bot).
  - One of them will need to be inverted

## Solenoids
N/A
  

## Sensors
- Falcon built in encoders


## Closed loop
Using built in Falcon 500 encoders.

We will be using flywheel RPM to adjust distance and height of shots (probably have a lookup table of distance v. RPM).

### Units
- Position: N/A
- Velocity: RPM
- Acceleration: RPM/s^2


## SysId
- Need to find kS, kV (kA probably doesn't matter)

