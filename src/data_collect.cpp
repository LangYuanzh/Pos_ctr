/** @file position_control.cpp
 *  @version 
 *  @date  2019.2.22
 *
 *  @brief
 *  Use Gear button chose gps or uwb
 *  Use mode control button obtain or realese control authority
 *
 *  @copyright 2019 Zhang. All rights reserved.
 *
 */

#include "uwbgps_control.h"
#include "dji_sdk/dji_sdk.h"
#include <fstream>
#include <stdio.h>    
#include <sys/time.h> 

//#include <fstream>
const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlRollPitchYawPub;
ros::Publisher ctrlBrakePub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Point current_local_pos;
geometry_msgs::Vector3 current_velocity;
geometry_msgs::Vector3 accel_current;
geometry_msgs::Point gps_current_pos;
geometry_msgs::Vector3 gps_current_velocity;
geometry_msgs::Point uwb_current_pos;
geometry_msgs::Vector3 uwb_current_velocity;

sensor_msgs::Joy rc;



std::ofstream attlog, accellog, heightlog, vellog, rclog;
char att[100] = {0},accel[100] = {0},gps[100] = {0},vel[100] = {0},rcl[100] = {0};
Mission square_mission;
void accel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

long long getCurrentTime();
long long getCurrentTime()    
{    
   struct timeval tv;    
   gettimeofday(&tv,NULL);    
   return (long long)tv.tv_sec * 1000 + tv.tv_usec / 1000;    
} 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_collect");
  ros::NodeHandle nh;
  
  
  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);// quaternion from FLU to ENU, 100Hz
  ros::Subscriber accelSub = nh.subscribe("dji_sdk/acceleration_ground_fused", 10, &accel_callback);// Fused acceleration with respect to East-North-Up (ENU) ground frame, published at 100 Hz.

  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);//only for height 50hz
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);//stop, on ground ,in air; 50Hz
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);//flight mode,50Hz
  ros::Subscriber VelocityENUSub = nh.subscribe("dji_sdk/velocity", 10, &velocity_callback);//50Hz only for velocity in z direction
  
  ros::Subscriber rcSub = nh.subscribe("dji_sdk/rc", 10, &rc_callback);//rc input // 50hz
  
  
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");//takeoff landing gohome
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");//
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");//Set the origin of the local position to be the current GPS coordinate. Fail if GPS health is low (<=3).

  time_t currtime = time(NULL);
	tm* p = localtime(&currtime);
	
 
  sprintf(att,"/home/xuanlingmu/flightlog/%d%02d%02d%02d%02d%02datt.txt",p->tm_year+1900,p->tm_mon+1,p->tm_mday,p->tm_hour,p->tm_min,p->tm_sec);
  sprintf(accel,"/home/xuanlingmu/flightlog/%d%02d%02d%02d%02d%02daccel.txt",p->tm_year+1900,p->tm_mon+1,p->tm_mday,p->tm_hour,p->tm_min,p->tm_sec);
  sprintf(gps,"/home/xuanlingmu/flightlog/%d%02d%02d%02d%02d%02dgps.txt",p->tm_year+1900,p->tm_mon+1,p->tm_mday,p->tm_hour,p->tm_min,p->tm_sec);
  sprintf(vel,"/home/xuanlingmu/flightlog/%d%02d%02d%02d%02d%02dvel.txt",p->tm_year+1900,p->tm_mon+1,p->tm_mday,p->tm_hour,p->tm_min,p->tm_sec);
  sprintf(rcl,"/home/xuanlingmu/flightlog/%d%02d%02d%02d%02d%02drc.txt",p->tm_year+1900,p->tm_mon+1,p->tm_mday,p->tm_hour,p->tm_min,p->tm_sec);
 
  //attlog.open(att);
  //attlog<<getCurrentTime();
  //attlog.close();
 
  //log.open("/home/xuanlingmu/flightlog.txt");
  //log.close();
  
  
  
  ros::spin();
  return 0;
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/
void
localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin)
{
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;

  deltaNed.y = deltaLat * deg2rad * C_EARTH;
  deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
  deltaNed.z = target.altitude - origin.altitude;
}


geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}



void calculateTargetAttFromAccel(geometry_msgs::Vector3 _target_accel)
{
    float acce_forward, acce_left;
    double att_yaw = toEulerAngle(current_atti).z;
    
}
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
  attlog.open(att,std::ios::app|std::ios::out);
  attlog<<getCurrentTime()<<" "<<toEulerAngle(current_atti).x<<" "<<toEulerAngle(current_atti).y<<" "<<toEulerAngle(current_atti).z<<" "
        <<accel_current.x<<" "<<accel_current.y<<" "<<accel_current.z<<std::endl;
        attlog.close();
}
void accel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
   accel_current = msg->vector;
}


void rc_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
  rc = *msg;
  rclog.open(rcl,std::ios::app|std::ios::out);
  rclog<<getCurrentTime()<<" "<<rc.axes[0]<<" "<<rc.axes[1]<<' '<<rc.axes[2]<<' '<<rc.axes[3]<<" "<<rc.axes[4]<<" "
       <<gps_current_velocity.x<<" "<<gps_current_velocity.y<<" "<<gps_current_velocity.z<<" "
       <<current_gps.altitude
       <<std::endl;
  rclog.close();
}
void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  gps_current_velocity = msg->vector;
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  current_gps = *msg;

}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}

bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}
