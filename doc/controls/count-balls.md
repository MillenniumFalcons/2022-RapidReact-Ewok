# Ball Counting
draft of ways to count and keep track of how many balls are in the robot. Assuming Ball stopped with existing beam break sensor on robot, 

# 2 Beam Breaks (one on top of column one at bottom)
![alt text](https://github.com/MillenniumFalcons/2022-RapidReact/blob/main/doc/reference-pictures/intakeColumnTopBottom.png)

## Logic Explaination ##
* 1 Ball enters through column, triggers bottom sensor:
    * Bottom: TRUE
    * Top: FALSE
    * Recorded Number of Balls: 1
    * Actual Number of Balls: 1
* 1 Ball travels through column, reaches middle, untriggers top sensor:
    * Bottom: FALSE
    * Top: FALSE
    * Recorded Number of Balls: 1
    * Actual Number of Balls: 1
* 1 Ball travels through column, reaches top, triggers top sensor:
    * Bottom: FALSE
    * Top: TRUE
    * Recorded Number of Balls: 1
    * Actual Number of Balls: 1
* 1 Ball enters bottom sensor as there is ball in top sensor, triggers bottom sensor, triggers top sensor:
    * Bottom: TRUE
    * Top: TRUE
    * Recorded Number of Balls: 2
    * Actual Number of Balls: 2
* (Problem) 1 Ball enters right next to another ball through the column, triggers the bottom sensor:
    * Bottom: TRUE
    * TOP: FALSE
    * Recorded Number of Balls: 1
    * Actual Number of Balls: 2
* (Problem) 1 Ball travels right next to another ball through column, reaches top, untriggers bottom, triggers top:
    * Bottom: FALSE
    * TOP: True
    * Recorded Number of Balls: 1
    * Actual Number of Balls: 2

## Simple Explaination ##
* Beam break trigger on bottom means a ball left/entered the robot.

# 2 Switches (one on top of column one at bottom)
![alt text](https://github.com/MillenniumFalcons/2022-RapidReact/blob/main/doc/reference-pictures/switchMount.png)

## Logic Explaination ## 
* 1 Ball enters through column, triggers bottom switch:
    * Bottom: TRUE
    * TOP: FALSE
    * Recorded Number of Balls: 1
    * Actual Number of Balls 1:
* 1 Ball travels through column, reaches middle, untriggers bottom switch:
    * Bottom: FALSE
    * TOP: FALSE
    * Recorded Number of Balls: 1
    * Actual Number of Balls 1: 
* 1 Ball travels through column, reaches top, triggers top switch:
    * Bottom: FALSE
    * TOP: TRUE
    * Recorded Number of Balls: 1
    * Actual Number of Balls 1: 
* 1 Ball enters through column, 1 Ball at the top of column, triggers bottom switch, triggers top switch:
    * Bottom: TRUE
    * Top: TRUE
    * Recorded Number of Balls: 2
    * Actual Number of Balls: 2
* 1 Ball travels right next to another ball through column, reaches top, triggers bottom switch:
    * Bottom: TRUE
    * Top: FALSE
    * Bottom: FALSE
    * Top: FALSE
    * Bottom: TRUE
    * TOP: FALSE
    * Recorded Number of Balls: 2
    * Actual Number of Balls: 2

## Simple Explaination ##
* Switch (more like buttons) triggers when ball runs over it in indexer/column. Use enum of indexer/column state to check if the bottom button is for adding balls or removing balls from the ball count. (If spitting balls out, the bottom triggers will mean balls are removed). Top switch will count how many balls got removed, bottom will count how many balls entered the robot. The balls that get sorted out b/c color will not matter since the switch will be inside the column, where the balls are sorted into the robot.

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
* Sensors placed on only top 1/4 of the ball so that consecutive balls still get counted seperately because of the gap at the top of two balls. Rest of logic works the same way as the switches logic
* Below is a picture of the ball and sensor explaination for clarity:
![alt text](https://github.com/MillenniumFalcons/2022-RapidReact/blob/main/doc/reference-pictures/ballFourthSensor.png)

# (mega brain) Ball tracking/Vision
![alt text](https://github.com/MillenniumFalcons/2022-RapidReact/blob/main/doc/reference-pictures/cameraCount.png)

## Simple Expaination ## 
* something with tracking the balls with vision...not sure yet