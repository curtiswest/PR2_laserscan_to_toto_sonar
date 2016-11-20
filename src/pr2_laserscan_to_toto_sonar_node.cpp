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

/*  In the Tiago we receive a LaserScan that we are going to use to
   simulate Totos sonar sensors.
   In Tiago, -1.98 is the extreme right measurement while 1.98 is extreme left measurement
   MiPAL sonar values are in cm and in a bit, so maximum range is 255
   Sonar 5 is the first 30 degress (pi/6) from -pi/2 to -pi/2+pi/6 = -pi/3
   Sonar 4 is the next wedge 30 degress (pi/6) from -pi/3 to -pi/6 
   Sonar 3 is the next wedge 30 degress (pi/6) from -pi/6 to  0
   Sonar 2 is the next wedge 30 degress (pi/6) from  0 to pi/6
   Sonar 1 is the next wedge 30 degress (pi/6) from   pi/6 to pi/3
*/

class ToToSONARS {

public:
   ToToSONARS (ros::NodeHandle n )
    {
        sub_sonar = n.subscribe("sonar_base", 1000, &ToToSONARS::sonarCallback,this);
        sub_scan = n.subscribe("scan", 1000,  &ToToSONARS::scanCallback,this);
        for(int i = Sonar::Left0; i < Sonar::NUMBER_OF_READINGS; i++)
           sensors.set_sonar(uint8_t(0), i);
        SENSORSSonarSensors_t wb_handler;               // whiteboard
        wb_handler.post(sensors);  
    }

void sonarCallback (const sensor_msgs::Range::ConstPtr& sonar_in)
    { std::stringstream ss; 
      ss << sonar_in->header.frame_id;
      std::string the_id (ss.str());
      std::string sonar_name_back_right ("3");
      std::string sonar_name_back ("2");
      std::string sonar_name_back_left ("1");
      std::size_t found = the_id.find(sonar_name_back_right);
      int sonar_id=8;
      if (found!=std::string::npos)
        {  sonar_id=7;
        }
      else
	{ found = the_id.find(sonar_name_back_left);
          if (found!=std::string::npos)
             {  sonar_id=10; }
	}

      toto_sonars[sonar_id]=100.0*sonar_in->range;
      uint8_t sonar_as_int = toto_sonars[sonar_id] >255? 255 : uint8_t(toto_sonars[sonar_id]);

      if (7==sonar_id)
           { sensors.set_sonar(sonar_as_int, Sonar::sSeven);
             ROS_INFO("s7 value: [%d]",sensors.sonar(Sonar::sSeven) );
           }
       else if (10==sonar_id)
           { sensors.set_sonar(sonar_as_int, Sonar::sTen);
             ROS_INFO("s10 value: [%d]",sensors.sonar(Sonar::sTen) );
           }
       else
	{ sensors.set_sonar(sonar_as_int, Sonar::sEight);
          ROS_INFO("s8 value: [%d]",sensors.sonar(Sonar::sEight) );

          sensors.set_sonar(sonar_as_int, Sonar::sNine);
          ROS_INFO("s9 value: [%d]",sensors.sonar(Sonar::sNine) );
	}
      //ROS_INFO(" Sonar %d: [%f]  id [%s]", sonar_id, sonar_n->range, (ss.str()).c_str()  ); 
    }


void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  int n_measurements[TOTO_SONARS];
  for (int i=0; i< TOTO_SONARS; i++)
	{ // skip values obtained from back sonar
          if ((7!=i) && (8!=i) && (9!=i) && (10!=i))
              toto_sonars[i]=255.0;
	  n_measurements[i]=0;
	}

  ROS_INFO("I heard: [%f]", scan_in->angle_min); ROS_INFO("I heard: [%f]", scan_in->angle_max);
  
  bool limit = false;
  double  theLaserAperture= scan_in->angle_min;
  int i=0;
  int sonar_id=6;
  while (theLaserAperture < scan_in->angle_max)
  {  
	/* s6 */
    if ( theLaserAperture <= -myPi/2.0  )
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
    else if ( myPi/2.0 < theLaserAperture ) 
		{  sonar_id=11; }

     if ((7!=i) && (8!=i) && (9!=i) && (10!=i))
         toto_sonars[sonar_id] =  scan_in->ranges[i] < toto_sonars[sonar_id] ? scan_in->ranges[i] : toto_sonars[sonar_id];
     n_measurements[sonar_id]+=1;
     i++;
     theLaserAperture+= scan_in->angle_increment;
  }

  // convert meters to centimiters
  for (int i=0; i< TOTO_SONARS; i++)
	{ if ((7!=i) && (8!=i) && (9!=i) && (10!=i))
            toto_sonars[i]=100.0*toto_sonars[i];
                   //ROS_INFO("s4 count: [%d] value: %f",s4_measureemnts,s4 );
	}

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

protected:
  ros::Subscriber sub_sonar;
  ros::Subscriber sub_scan;
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
