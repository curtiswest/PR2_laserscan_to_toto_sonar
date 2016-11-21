pr2_laserscan_to_toto_sonar converts the sensor_msgs/LaserScan.msg topics coming from the PR2 into the equivalent sonar sensors found on the Toto robot. There are two variants described below, decide which suits your needs.

#Laserscan
This is the original variant, and provides full 30-deg coverage in sonar regions 0 through 6 & 11, but only 10-deg coverage in regions 7 & 10. However, you do not need the PR2 variant with the rear laser scanner, and as such you can run it with the stock pr2_gazebo launch files (although you will need to find your own way to launch it into a world etc.).

However, if the rear_laser is available on topic /rear_scan, then this is used to generate regions 8 & 9 with full coverage only. If this topic is not available, 8 and 9 simply read the max 255cm value and can be safely ignored.

Execute it with:

	`./devel/lib/pr2_laserscan_to_toto_sonar/pr2_laserscan_to_toto_sonar_node`

##Full Range Laser Scan
This variant utilises the rear laser scan to get full coverage of all sonar regions, including 7 & 10.  The front laser scanner covers ranges 0 through 6 & 11, while the rear covers the remainder. Note that due to the layout of the laser scanners, there may be overlap in the regions of the sonar so you may need to account for this. 

Execute it with:

	`./devel/lib/pr2_laserscan_to_toto_sonar/pr2_fullrangelaserscan_to_toto_sonar_node`
