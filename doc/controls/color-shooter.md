# 1 Color Sensor next to kciker belt/acceleration area or next to banner sensor
Will slow down flywheel and kickers to fraction of set rpm to ensure wrong colored ball doesn't get shot

## Logic Explaination
* One correct colored ball gets indexed at bottom of column
    * getColor(): UNKNOWN
    * columnBottom.getBannerSensorValue(): false
    * continues to run intake, bottom column, and indexer
* One correct colored ball reaches top of bottom column
    * getColor(): correct color
    * columnBottom.getBannerSensorValue(): true
    * continues running all subsystems like normal, shoot if needed
* One incorrect color ball gets indexed at the bottom of column
    * getColor(): UNKNOWN
    * columnBottom.getBannerSensorValue(): false
    * continues indexing and move up to column like normal
* one incorrect ball reaches top of bottom column
    * getColor(): correct color
    * columnBottom.getBannerSensorValue(): true
    * setFlywheelRPM multiplied by a fraction to reduce speed, revert once the ball has been shot (from ball counting based on banner sensors)
    