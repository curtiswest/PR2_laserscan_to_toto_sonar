#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include "typeClassDefs/SENSORSSonarSensors.h"
#include "gugenericwhiteboardobject.h"
#include "guwhiteboardtypelist_generated.h"
#include <string>    
#include <sstream>    

#define myPi 3.141592
#define TOTO_SONARS 12

using namespace guWhiteboard;

/* From the PR2, we recieve two sensor_msgs/LaserScan messages:
 * 	/base_scan - from the laser at the base of the robot
 * 	/tilt_scan - from the pan-tilt laser at shoulder height
 * We will use the /base_scan LaserScan here as it covers the front ~260 deg of the robot, which means that
 * 	extreme right measurement: -2.2689 rad
 * 	extreme left measurement :  2.2689 rad
 * The LaserScan ranges to match Toto's sonar sensors are:
	*0: pi/2 	to 	2pi/6
	*1: 2pi/6	to	pi/6
	*2: pi/6	to	0
	*3: 0		to	-pi/6
	*4: -pi/6	to	-2pi/6
	*5: -2pi/6	to	-pi/2
	*6: -pi/2	to	-4pi/6
	*7:	-4pi/6 	to	-4pi/6 + pi/18 (only 1/3rd sweep angle)
	*	<blank zone w/o sensors>
	*10: 11pi/18 to 4pi/6 (only 1/3rd sweep angle)
	*11: 4pi/6	to	pi/2
 *
 */

class ToToSONARS {

public:
   ToToSONARS (ros::NodeHandle n )
    {
        // sub_sonar = n.subscribe("sonar_base", 1000, &ToToSONARS::sonarCallback,this); // TODO:: Implement /tilt_scan here
        sub_front_scan = n.subscribe("base_scan", 1000,  &ToToSONARS::frontScanCallback,this);
        sub_rear_scan = n.subscribe("rear_scan", 1000,  &ToToSONARS::rearScanCallback,this);
        for(int i = Sonar::Left0; i < Sonar::NUMBER_OF_READINGS; i++)
           sensors.set_sonar(uint8_t(0), i);
        SENSORSSonarSensors_t wb_handler; //gusimplewhiteboard
        wb_handler.post(sensors);  
    }


/*
 * Convert front laser scan (/base_scan) into sonar reasons 0-6 & 11.
 */
void frontScanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  int n_measurements[TOTO_SONARS];
  for (int i=0; i< TOTO_SONARS; i++)
	{ // skip values obtained from back laser scanner
          if ((7!=i) && (8!=i) && (9!=i) && (10!=i))
              toto_sonars[i]=255.0;
	  n_measurements[i]=0;
	}

  ROS_INFO("NEW FRONT SCAN: Min angle [%f] | Max angle [%f]", scan_in->angle_min, scan_in->angle_max);
  
  //bool limit = false;
  double  theLaserAperture= scan_in->angle_min;
  int i=0;
  int sonar_id=7;

  //Loop through each LaserScan angle_increment and adjust reading based on each
  while (theLaserAperture < scan_in->angle_max)
  {  
	  if ( theLaserAperture <= -myPi/2.0 && theLaserAperture > -4*myPi/6.0)
		{  sonar_id=6; }
	/* s5 */
    else if (( -myPi/2.0 <= theLaserAperture ) && ( theLaserAperture <= -myPi/3.0 ) )
		{  sonar_id=5; }
	/* s4 */
    else if (( -myPi/3.0 <= theLaserAperture ) && ( theLaserAperture <= -myPi/6.0 ) )
		{  sonar_id=4; }
	/* s3 */
    else if (( -myPi/6.0 < theLaserAperture ) && ( theLaserAperture <= 0.0 ) )
		{  sonar_id=3; }
	/* s2 */
    else if (( 0.0 < theLaserAperture ) && ( theLaserAperture <= myPi/6.0 ) )
		{  sonar_id=2; }
	/* s1 */
    else if (( myPi/6.0 < theLaserAperture ) && ( theLaserAperture <= myPi/3.0 ) )
		{  sonar_id=1; }
	/* s0 */
    else if (( myPi/3.0 < theLaserAperture ) && ( theLaserAperture <= myPi/2.0 ) )
		{  sonar_id=0; }
	/* s11 */
    else if ( myPi/2.0 < theLaserAperture && 4*myPi/6.0 >= theLaserAperture )
		{  sonar_id=11; }

	if ((7!=sonar_id) && (8!=sonar_id) && (9!=sonar_id) && (10!=sonar_id)) {
	         toto_sonars[sonar_id] =  scan_in->ranges[i] < toto_sonars[sonar_id] ? scan_in->ranges[i] : toto_sonars[sonar_id];
	}
	n_measurements[sonar_id]+=1;
	i++;
	theLaserAperture+= scan_in->angle_increment;
  }

  // convert meters to centimeters
  for (int i=0; i< TOTO_SONARS; i++)
	{ if ((7!=i) && (8!=i) && (9!=i) && (10!=i))
            toto_sonars[i]=100.0*toto_sonars[i];
                   //ROS_INFO("s4 count: [%d] value: %f",s4_measureemnts,s4 );
	}

  //Cap sonar reading to 255 as per MiPAL sonar reading standard
   uint8_t s0_as_int = toto_sonars[0] >255? 255 : uint8_t(toto_sonars[0]);
   sensors.set_sonar(s0_as_int, Sonar::sZero);
   ROS_INFO("s0 value: [%d]",sensors.sonar(Sonar::sZero) );

   uint8_t s1_as_int = toto_sonars[1] >255? 255 : uint8_t(toto_sonars[1]);
   sensors.set_sonar(s1_as_int, Sonar::sOne);
   ROS_INFO("s1 value: [%d]",sensors.sonar(Sonar::sOne) );

   uint8_t s2_as_int = toto_sonars[2] >255? 255 : uint8_t(toto_sonars[2]);
   sensors.set_sonar(s2_as_int, Sonar::sTwo);
   ROS_INFO("s2 value: [%d]",sensors.sonar(Sonar::sTwo) );

   uint8_t s3_as_int = toto_sonars[3] >255? 255 : uint8_t(toto_sonars[3]);
   sensors.set_sonar(s3_as_int, Sonar::sThree);
   ROS_INFO("s3 value: [%d]",sensors.sonar(Sonar::sThree) );

   uint8_t s4_as_int = toto_sonars[4] >255? 255 : uint8_t(toto_sonars[4]);
   sensors.set_sonar(s4_as_int, Sonar::sFour);
   ROS_INFO("s4 value: [%d]",sensors.sonar(Sonar::sFour) );

   uint8_t s5_as_int = toto_sonars[5] >255? 255 : uint8_t(toto_sonars[5]);
   sensors.set_sonar(s5_as_int, Sonar::sFive);
   ROS_INFO("s5 value: [%d]",sensors.sonar(Sonar::sFive) );

   uint8_t s6_as_int = toto_sonars[6] >255? 255 : uint8_t(toto_sonars[6]);
   sensors.set_sonar(s6_as_int, Sonar::sSix);
   ROS_INFO("s6 value: [%d]",sensors.sonar(Sonar::sSix) );

   uint8_t s11_as_int = toto_sonars[11] >255? 255 : uint8_t(toto_sonars[11]);
   sensors.set_sonar(s11_as_int, Sonar::sEleven);
   ROS_INFO("s11 value: [%d]",sensors.sonar(Sonar::sEleven) );

   SENSORSSonarSensors_t wb_handler;               // whiteboard
   wb_handler.post(sensors);  
}


/*
 * Convert rear laser scan (/rear_scan) into sonar regioons 7-10.
 */
void rearScanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  int n_measurements[TOTO_SONARS];
  for (int i=0; i< TOTO_SONARS; i++)
	{ // skip values obtained from front sonar
          if ((7==i) || (8==i) || (9==i) || (10==i))
              toto_sonars[i]=255.0;
	  n_measurements[i]=0;
	}

  ROS_INFO("NEW REAR SCAN: Min angle [%f] | Max angle [%f]", scan_in->angle_min, scan_in->angle_max);

  //bool limit = false;
  double  theLaserAperture= scan_in->angle_min;
  int i=0;
  int sonar_id=0;

  //Loop through each LaserScan angle_increment and adjust reading based on each
  while (theLaserAperture < scan_in->angle_max) {
	 if(theLaserAperture >= myPi/6.0) {
		 sonar_id = 7;
	 } else if (myPi/6.0 >= theLaserAperture && theLaserAperture > 0) {
		 sonar_id = 8;
	 } else if (0 >= theLaserAperture && theLaserAperture > -myPi/6.0) {
		 sonar_id = 9;
	 } else if (theLaserAperture < -myPi/6.0) {
		 sonar_id = 10;
	 }

	//Keep only the shortest distance for this sonar range
	if ((7==sonar_id) || (8==sonar_id) || (9==sonar_id) || (10==sonar_id)) {
			//ROS_INFO("Sonar #[%d] with value [%f]", sonar_id, scan_in->ranges[i]);
	         toto_sonars[sonar_id] =  scan_in->ranges[i] < toto_sonars[sonar_id] ? scan_in->ranges[i] : toto_sonars[sonar_id];
	}
	n_measurements[sonar_id]+=1;
	i++;
	theLaserAperture+= scan_in->angle_increment;
  }

  // convert meters to centimeters
  for (int i=0; i< TOTO_SONARS; i++)
	{ if ((7==i) || (8==i) || (9==i) || (10==i))
            toto_sonars[i]=100.0*toto_sonars[i];
                   //ROS_INFO("s4 count: [%d] value: %f",s4_measureemnts,s4 );
	}

  //Cap sonar reading to 255 as per MiPAL sonar reading standard
   uint8_t s7_as_int = toto_sonars[7] >255? 255 : uint8_t(toto_sonars[7]);
   sensors.set_sonar(s7_as_int, Sonar::sSeven);
   ROS_INFO("s7 value: [%d]",sensors.sonar(Sonar::sSeven) );

   uint8_t s8_as_int = toto_sonars[8] >255? 255 : uint8_t(toto_sonars[8]);
   sensors.set_sonar(s8_as_int, Sonar::sEight);
   ROS_INFO("s8 value: [%d]",sensors.sonar(Sonar::sEight) );

   uint8_t s9_as_int = toto_sonars[9] >255? 255 : uint8_t(toto_sonars[9]);
   sensors.set_sonar(s9_as_int, Sonar::sNine);
   ROS_INFO("s9 value: [%d]",sensors.sonar(Sonar::sNine) );

   uint8_t s10_as_int = toto_sonars[10] >255? 255 : uint8_t(toto_sonars[10]);
   sensors.set_sonar(s10_as_int, Sonar::sTen);
   ROS_INFO("s10 value: [%d]",sensors.sonar(Sonar::sTen) );

   SENSORSSonarSensors_t wb_handler;               // whiteboard
   wb_handler.post(sensors);
}


protected:
  ros::Subscriber sub_rear_scan;
  ros::Subscriber sub_front_scan;

  SENSORSSonarSensors   sensors;                  // sensor values
  double toto_sonars[TOTO_SONARS];
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_to_sonar");
  ros::NodeHandle n;
  ToToSONARS subscribers(n);

  ros::spin();

  return 0;
}
