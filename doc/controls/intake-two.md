# Use banner sensor
* If there is ball at top of column, stop running column, hold ball in the indexing area until top ball is shot, then run column again.
* Solves issue of shooting one ball at a time as well

## Logic Explaination
* One ball at top of column, bottom ball gets intaked
    * columnBottom.getBannerSensorValue(): true
    * column stopped, ball intaked and held in indexing area
* One ball at top of column, bottom ball is in indexing area, shooting top ball
    * indexer and column run to get top ball into acceleration area, get bottom ball into column area
    * repeat process for shooting and indexing balls
    