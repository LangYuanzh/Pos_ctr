/** @file demo_flight_control.h
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use flight control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */
 /**
 * local_position : Cartesian ENU frame, origin:set by calling set_local_pos_ref service ,need GPS
 */

#ifndef DEMO_FLIGHT_CONTROL_H
#define DEMO_FLIGHT_CONTROL_H

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include <math.h>
// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>


#include <tf/tf.h>
#include <sensor_msgs/Joy.h>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

double alt_P;
double vz_P;
double vz_I;
double vz_D;
  
double xy_P;
double vxy_P;
double vxy_I;
double vxy_D;

double target_X,target_Y,target_Z;
double max_pitch_roll;

/*!
 * @brief a bare bone state machine to track the stage of the mission
 */

class Mission
{
public:
  // The basic state transition flow is:
  // 0---> 1 ---> 2 ---> ... ---> N ---> 0
  // where state 0 means the mission is note started
  // and each state i is for the process of moving to a target point.
  int state;

  int inbound_counter;
  int outbound_counter;
  int break_counter;

  float target_yaw;

  
  
  geometry_msgs::Vector3 target_position;
  geometry_msgs::Vector3 target_offset;
  geometry_msgs::Vector3 target_velocity;
  geometry_msgs::Vector3 target_accel;
  geometry_msgs::Vector3 target_attitude;

  
  geometry_msgs::Vector3 velocity_ENU;
  geometry_msgs::Vector3 velocity_offset;

  geometry_msgs::Vector3 intergrated;
  geometry_msgs::Vector3 derivative;
  geometry_msgs::Vector3 old_vel_err;
  sensor_msgs::NavSatFix start_gps_location;
  geometry_msgs::Point start_local_position;

  bool finished;

  Mission() : state(0), inbound_counter(0), outbound_counter(0), break_counter(0),
              finished(false)
  {
    intergrated.x = 0.0;
    intergrated.y = 0.0;
    intergrated.z = 0.0;
    derivative.x = 0.0;
    derivative.y = 0.0;
    derivative.z = 0.0;
    old_vel_err.x = 0.0;
    old_vel_err.y = 0.0;
    old_vel_err.z = 0.0;

  }

  void step();

  void setTarget(float x, float y, float z, float yaw)
  {
    target_position.x = x;
    target_position.y = y;
    target_position.z = z;
    target_yaw      = yaw;
  }
  
  void calculateDesVel(geometry_msgs::Vector3 _target_offset);
  void calculateDesAccel(geometry_msgs::Vector3 _velocity_offset);
  void calculateTargetAttFromAccel(geometry_msgs::Vector3 _target_accel);

  void reset()
  {
    inbound_counter = 0;
    outbound_counter = 0;
    break_counter = 0;
    finished = false;
  }

};

void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin);

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

void rc_callback(const sensor_msgs::Joy::ConstPtr& msg);

bool takeoff_land(int task);

bool obtain_control();

bool realease_control();

bool is_M100();

bool monitoredTakeoff();

bool M100monitoredTakeoff();

bool set_local_position();

#endif // DEMO_FLIGHT_CONTROL_H
