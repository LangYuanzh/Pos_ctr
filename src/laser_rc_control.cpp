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
#include <iostream>
#include <sys/time.h> 
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

geometry_msgs::Point gps_current_pos;
geometry_msgs::Vector3 gps_current_velocity;

sensor_msgs::Joy rc;

bool usegps = true;

bool obtainControlPermission = false;
bool obtain_control_result = false;
float rc_des_vel_z = 0;
char logname[100] = {0};
Mission square_mission;

long getCurrentTime();
long getCurrentTime()    
{    
   struct timeval tv;    
   gettimeofday(&tv,NULL);    
   return tv.tv_sec * 1000 + tv.tv_usec / 1000;    
} 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_rc_control");
  ros::NodeHandle nh;
  
  
  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);// quaternion from FLU to ENU, 100Hz
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);//Fused global position in lat, long and alt(m). Position in WGS 84 , 50 Hz.
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);//stop, on ground ,in air; 50Hz
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);//flight mode,50Hz

  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
  ros::Subscriber VelocityENUSub = nh.subscribe("dji_sdk/velocity", 10, &velocity_callback);//50Hz

  ros::Subscriber rcSub = nh.subscribe("dji_sdk/rc", 10, &rc_callback);
  // Publish the control signal
  ctrlRollPitchYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 10);
  
  // We could use dji_sdk/flight_control_setpoint_ENUvelocity_yawrate here, but
  // we use dji_sdk/flight_control_setpoint_generic to demonstrate how to set the flag
  // properly in function Mission::step()
  ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");//takeoff landing gohome
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");//
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");//Set the origin of the local position to be the current GPS coordinate. Fail if GPS health is low (<=3).

  nh.param("/laser_rc_control/alt_P", alt_P, 1.3);
  nh.param("/laser_rc_control/vz_P",  vz_P,  0.5);
  nh.param("/laser_rc_control/vz_I",  vz_I,  0.05);
  nh.param("/laser_rc_control/vz_D",  vz_D,  0.2);
  nh.param("/laser_rc_control/xy_P",  xy_P,  1.0);
  nh.param("/laser_rc_control/vxy_P", vxy_P, 0.85);
  nh.param("/laser_rc_control/vxy_I", vxy_I, 0.45);
  nh.param("/laser_rc_control/vxy_D", vxy_D, 0.11);
  nh.param("/laser_rc_control/tag_X", target_X, 0.11);
  nh.param("/laser_rc_control/tag_Y", target_Y, 0.11);
  nh.param("/laser_rc_control/tag_Z", target_Z, 0.11);
  nh.param("/laser_rc_control/max_V", max_V, 3.0);
  nh.param("/laser_rc_control/max_ACC", max_ACC, 1.5);
  nh.param("/laser_rc_control/max_pitch_roll", max_pitch_roll, 10.0);
  
  time_t currtime = time(NULL);
	tm* p = localtime(&currtime);
  sprintf(logname,"/home/xuanlingmu/flightlog/%d%02d%02d%02d%02d%02d.txt",p->tm_year+1900,p->tm_mon+1,p->tm_mday,p->tm_hour,p->tm_min,p->tm_sec);
  //ROS_ERROR("????????????????????????????   %f %f %f",target_X,target_Y,target_Z);
  //std::fstream lognamefile;
  //square_mission.log.open(logname,std::ios::app|std::ios::out);
  if (!set_local_position()) // We need this for height
  {
	ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
	return 0;
  }		  	
  //log.open("/home/xuanlingmu/flightlog.txt");
  //log.close();
  
  ROS_INFO("waiting for permisssion..............!");
  
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

void Mission::step()
{
  static int info_counter = 0;
  geometry_msgs::Vector3     localOffset;

  float speedFactor         = 2;
  float yawThresholdInDeg   = 2;

  float rollCmd, pitchCmd, zCmd;

  localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);
/*
  double xOffsetRemaining = target_offset_x - localOffset.x;
  double yOffsetRemaining = target_offset_y - localOffset.y;
  double zOffsetRemaining = target_offset_z - localOffset.z;

  double yawDesiredRad     = deg2rad * target_yaw;
  double yawThresholdInRad = deg2rad * yawThresholdInDeg;
  double yawInRad          = toEulerAngle(current_atti).z;
*/
  target_offset.x = target_position.x - current_local_pos.x;
  target_offset.y = target_position.y - current_local_pos.y;
  target_offset.z = target_position.z - current_local_pos.z;
  calculateDesVel(target_offset);
                              
  velocity_offset.x = target_velocity.x - current_velocity.x;
  velocity_offset.y = target_velocity.y - current_velocity.y;
  velocity_offset.z = target_velocity.z - current_velocity.z;
  calculateDesAccel(velocity_offset);

  calculateTargetAttFromAccel(target_accel);
  
  rollCmd  = target_attitude.x;
  pitchCmd = target_attitude.y;
  zCmd     = rc_des_vel_z;
  
  double attyaw = toEulerAngle(current_atti).z;
  log.open(logname,std::ios::app|std::ios::out);
  log <<gps_current_pos.x<<" "<<gps_current_pos.y<<" "<<gps_current_velocity.z<<" "<<rc_des_vel_z<<" "
      <<" "<<current_local_pos.x<<" "<<current_local_pos.y<<" "<<current_local_pos.z<<" "
      <<target_offset.x<<" "<<target_offset.y<<" "<<target_velocity.x<<" "<<target_velocity.y<<" "
      <<velocity_offset.x<<" "<<velocity_offset.y<<" "<<target_accel.x<<" "<<target_accel.y<<" "<<attyaw<<std::endl;   
  log.close();
   
  ROS_INFO("despos: %f ,%f, %f offset: %f, %f, %f currntpos: %f, %f, %f ",target_position.x,target_position.y,target_position.z,target_offset.x,target_offset.y,target_offset.z,current_local_pos.x,
  current_local_pos.y,current_local_pos.z);
  ROS_WARN("velerr: %f, %f, %f desacce: %f ,%f ,%f , desrollpich: %f, %f", velocity_offset.x,velocity_offset.y,velocity_offset.z,target_accel.x,target_accel.y,target_accel.z,
  target_attitude.x,target_attitude.y);

  info_counter++;
  if(info_counter > 25)
  {
    info_counter = 0;
    
  }

  if(target_offset.x>-0.15&&target_offset.x<0.15&&target_offset.y>-0.15&&target_offset.x<0.15)
  {
     static ros::Time starttime = ros::Time::now();
     ros::Duration elapsed_time = ros::Time::now() - starttime;
     if(elapsed_time > ros::Duration(60))
       {
          starttime = ros::Time::now();
          square_mission.finished = true;
          
       }
  }
  /*!
   * @brief: if we already started breaking, keep break for 50 sample (1sec)
   *         and call it done, else we send normal command
   */

  
  //if(break_counter > 0)
  //{
    sensor_msgs::Joy controlVelYawRate;
    uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                DJISDK::HORIZONTAL_ANGLE |
                DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_GROUND   |
                DJISDK::STABLE_ENABLE);
    controlVelYawRate.axes.push_back(rollCmd);
    controlVelYawRate.axes.push_back(pitchCmd);
    controlVelYawRate.axes.push_back(zCmd);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(flag);

    ctrlBrakePub.publish(controlVelYawRate);
    break_counter++;
    return;
  //}
  /*else //break_counter = 0, not in break stage
  {
    sensor_msgs::Joy controlRollPitchYaw;
    //ROS_ERROR("SEND SEND SEND");
    controlRollPitchYaw.axes.push_back(rollCmd);
    controlRollPitchYaw.axes.push_back(pitchCmd);
    controlRollPitchYaw.axes.push_back(target_Z);
    controlRollPitchYaw.axes.push_back(0);
    ctrlRollPitchYawPub.publish(controlRollPitchYaw);
  }*/

}
void rc_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
  rc = *msg;
  rc_des_vel_z = rc.axes[3];
  //mode control button: obtain or realese control authority
  if(rc.axes[4] == -10000)
     obtainControlPermission = true;
  if(rc.axes[4] ==  10000)
    obtainControlPermission = false; 

  if(obtainControlPermission && (!obtain_control_result))
	  {
		  obtain_control_result = obtain_control();
		  bool takeoff_result;
		    
		   ROS_INFO("A3/N3 taking off!");
		   takeoff_result = monitoredTakeoff();

		  if(takeoff_result)
		  {
			square_mission.reset();
			square_mission.start_gps_location = current_gps;
			square_mission.start_local_position = current_local_pos;
			square_mission.setTarget(target_X, target_Y, target_Z, 0);
			square_mission.state = 1;
			ROS_INFO("##### Start route %d ....", square_mission.state);
		  }
         
	  }
	  else if(obtain_control_result && (!obtainControlPermission))
		{
			bool realeasecontrol = realease_control();
			ROS_WARN("REALEASE CONTROL");
            obtain_control_result = false;
		}	

  
}
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  current_gps = *msg;

  // Down sampled to 50Hz loop
  if(elapsed_time > ros::Duration(0.02))
  {
    start_time = ros::Time::now();
    if(obtainControlPermission)
	 {	switch(square_mission.state)
		 {
        case 0:
          break;

        case 1:
          if(!square_mission.finished)
          {
            square_mission.step();
          }
          else
          {
            //square_mission.reset();
            //square_mission.start_gps_location = current_gps;
          // square_mission.start_local_position = current_local_pos;
            square_mission.setTarget(8, 0, 3, 0);
                square_mission.finished = false;
                square_mission.step();
            //square_mission.state = 2;
            ROS_INFO("##### Start route %d ....", square_mission.state);
          }
          break;
		  
		  }
    }
  }
}

void Mission::calculateDesVel(geometry_msgs::Vector3 _target_offset)
{
   float dis_target_max = 3.0;
   float dis_to_target = sqrt(_target_offset.x*_target_offset.x + _target_offset.y*_target_offset.y);
   if(dis_to_target > dis_target_max)
   {
       _target_offset.x = _target_offset.x * dis_target_max/dis_to_target;
       _target_offset.y = _target_offset.y * dis_target_max/dis_to_target;
   }
   
   target_velocity.x = _target_offset.x * xy_P;
   target_velocity.y = _target_offset.y * xy_P;
   if(target_velocity.x > max_V)target_velocity.x = max_V;
   if(target_velocity.x < -max_V)target_velocity.x = -max_V;
   if(target_velocity.y > max_V)target_velocity.y = max_V;
   if(target_velocity.y < -max_V)target_velocity.y = -max_V;

   if(_target_offset.z > 3)
       _target_offset.z = 3;
    else if(_target_offset.z < -3)
       _target_offset.z = -3;
    
    target_velocity.z = _target_offset.z * alt_P;  
}

void Mission::calculateDesAccel(geometry_msgs::Vector3 _velocity_offset)
{
    float px, py, ix, iy, dx, dy, pz, iz, dz;
   intergrated.x = _velocity_offset.x + old_vel_err.x;
   intergrated.y = _velocity_offset.y + old_vel_err.y;
   intergrated.z = _velocity_offset.z + old_vel_err.z;
   derivative.x = _velocity_offset.x - old_vel_err.x;
   derivative.y = _velocity_offset.y - old_vel_err.y;
   derivative.z = _velocity_offset.z - old_vel_err.z;
   
   px = vxy_P * _velocity_offset.x;
   ix = vxy_I * intergrated.x;
   dx = vxy_D * derivative.x; 
   target_accel.x = px + ix +dx;
   if(target_accel.x > max_ACC)target_accel.x = max_ACC;
   if(target_accel.x < -max_ACC)target_accel.x = -max_ACC;
   py = vxy_P * _velocity_offset.y;
   iy = vxy_I * intergrated.y;
   dy = vxy_D * derivative.y; 
   target_accel.y = py + iy +dy;
   if(target_accel.y > max_ACC)target_accel.y = max_ACC;
   if(target_accel.y < -max_ACC)target_accel.y = -max_ACC;
   pz = vz_P * _velocity_offset.z;
   iz = vz_I * intergrated.z;
   dz = vz_D * derivative.z;
   target_accel.z = pz + iz + dz;
   
   old_vel_err = _velocity_offset;
   
}

void Mission::calculateTargetAttFromAccel(geometry_msgs::Vector3 _target_accel)
{
    float acce_forward, acce_left;
    double att_yaw = toEulerAngle(current_atti).z;
    //att_yaw = att_yaw;// - 1.5707963265;
    double x,y;
    acce_forward = _target_accel.x * cos(att_yaw) + _target_accel.y * sin(att_yaw);
    acce_left = -_target_accel.x * sin(att_yaw) + _target_accel.y * cos(att_yaw);
    
    target_attitude.y = atan(acce_forward / 9.8);
    target_attitude.x = -atan(acce_left * cos(target_attitude.y) / 9.8);
    
    //ROS_INFO("yaw: %f ACC: %f, %f accbody: %f, %f att: %f, %f ", att_yaw, _target_accel.x,_target_accel.y,acce_forward,acce_left,target_attitude.x,target_attitude.y);
    if(target_attitude.y > (max_pitch_roll*C_PI/180.0))
        target_attitude.y =  max_pitch_roll*C_PI/180.0;
    else if (target_attitude.y < -(max_pitch_roll*C_PI/180.0))
        target_attitude.y =  -max_pitch_roll*C_PI/180.0;

    if(target_attitude.x > (max_pitch_roll*C_PI/180.0))
        target_attitude.x =  max_pitch_roll*C_PI/180.0;
    else if (target_attitude.x < -(max_pitch_roll*C_PI/180.0))
        target_attitude.x =  -max_pitch_roll*C_PI/180.0;   

}

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool realease_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=0;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("realease control failed!");
    return false;
  }

  return true;
}



void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
   gps_current_pos = msg->point;
    current_local_pos = gps_current_pos;
}

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  gps_current_velocity = msg->vector;

  if(usegps)
    current_velocity = gps_current_velocity;
}


void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}


/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(5)) {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }


  // Step 1.2: Get in to the air
  while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
          (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(20)) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}

bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}
