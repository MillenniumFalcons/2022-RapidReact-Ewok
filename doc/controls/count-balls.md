# 2 Beam Breaks (one top one bottom, but only detecet top 1/4 of the ball instead of the center of the ball)
![alt text](https://github.com/MillenniumFalcons/2022-RapidReact/blob/main/doc/reference-pictures/beamBreakFourth.png)

## Logic Explaination ##
* 1 Ball enter the column, triggers bottom sensor:
    * Bottom: TRUE
    * Top: FALSE
    * Recorded Number of Balls: 1
    * Actual Number of Balls: 1
* 1 Ball enter column, travels up column, untriggers bottom sensor:
    * Bottom: FALSE
    * Top: FALSE
    * Recorded Number of Balls: 1
    * Actual Number of Balls: 1
* 1 travel up column, reaches top sensor, triggers top sensor:
    * Bottom: FALSE
    * Top: TRUE
    * Recorded Number of Balls: 1
    * Actual Number of Balls: 1
* 1 Ball travels right next to another ball through column, triggers bottom sensor:
    * Bottom: TRUE
    * Top: FALSE
    * Bottom: FALSE
    * Top: FALSE
    * BOTTOM: TRUE
    * Top: FALSE
    * Recorded Number of Balls: 2
    * Actual Number of Balls: 2

## Simple Explaination ##
* Sensors placed on only top 1/4 of the ball so that consecutive balls still get counted seperately because of the gap at the top of two balls. Rest of logic works the same way as the switches logic. Adds/subtract balls by remember what the sensor state was in the previous loop, and if the state changed, add/subtract
* Below is a picture of the ball and sensor explaination for clarity:
![alt text](https://github.com/MillenniumFalcons/2022-RapidReact/blob/main/doc/reference-pictures/ballFourthSensor.png)
