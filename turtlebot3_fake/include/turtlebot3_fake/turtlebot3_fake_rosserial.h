/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo */

/*******************************************************************************
 * This file was copied from the turtlebot3_simulations repository.
 * 
 * The goal is to have a self-contained, compilable simulation that doesn't rely
 * on having the full ROS installation.
 * 
 * To support this, I am trying to leverage the turtlebot3_core files from
 * OpenCR.  Notably, these are already quite similar and I wouldn't be surprised
 * if one was derived from the other...
 * 
 * Modifier:  Michael Dodson
 ********************************************************************************/

#ifndef TURTLEBOT3_FAKE_ROSSERIAL_H_
#define TURTLEBOT3_FAKE_ROSSERIAL_H_

#include <math.h>

/* ros/ros.h is a file full of local includes and is conceptually
different from the one used by opencr.

commenting it out until we decide we need it. */
// #include <ros/ros.h>
#include <ros.h>  // from opencr

/* all of these are included in the opencr repo */
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
// #include <sensor_msgs/BatteryState.h>
// #include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <turtlebot3_msgs/SensorState.h>
#include <turtlebot3_msgs/Sound.h>
#include <turtlebot3_msgs/VersionInfo.h>

/* from opencr, TurtleBot3.h includes several other header files that define:
turtlebot3_motor_driver.h
turtlebot3_sensor.h
turtlebot3_controller.h
turtlebot3_diagnosis.h */

// #include <TurtleBot3.h>

// #include "turtlebot3_burger.h"  // from opencr.  unnecessary because all these things are defined in this file


/* not sure what this is doing in the original file from
turtlebot3_simulations repo.  seems to be self-referential.
leaving it here for now. */
// #include "turtlebot3_fake.h"

/* these are very similar to the definitions in turtlebot3_core_config.h:46-70 */
#define NAME "Burger"  // not sure if NAME is defined elsewhere
#define FIRMWARE_VER "1.2.3"

#define WHEEL_RADIUS                    0.033     // meter

#define WHEEL_NUM                       2
#define LEFT                            0
#define RIGHT                           1

#define MAX_LINEAR_VELOCITY             0.22   // m/s
#define MAX_ANGULAR_VELOCITY            2.84   // rad/s
#define VELOCITY_STEP                   0.01   // m/s
#define VELOCITY_LINEAR_X               0.01   // m/s
#define VELOCITY_ANGULAR_Z              0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X         1
#define SCALE_VELOCITY_ANGULAR_Z        1

#define DEG2RAD(x)                      (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                      (x * 57.2957795131)  // *180/PI

#define TORQUE_ENABLE                   1       // Value for enabling the torque of motor
#define TORQUE_DISABLE                  0       // Value for disabling the torque of motor


/*******************************************************************************
* Callback function prototypes
*******************************************************************************/
//void commandVelocityCallback(const geometry_msgs::TwistConstPtr cmd_vel_msg);
/* from opencr */
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);

/*******************************************************************************
* Function prototypes
*******************************************************************************/
void initOdom(void);  //copied from opencr
void initJointStates(void);  // copied from opencr

bool updateOdometry(ros::Duration diff_time);  // arguments are void in opencr
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateJoint(void);

// void resetCallback(const std_msgs::Empty& reset_msg);  // copied from opencr

void publishVersionInfoMsg(void);  // copied from opencr

ros::Time rosNow(void);  // from opencr
ros::Time addMicros(ros::Time & t, uint32_t _micros); // from opencr // deprecated

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;
ros::NodeHandle nh_priv;

/*******************************************************************************
* ROS Time
*******************************************************************************/
ros::Time last_cmd_vel_time;
ros::Time prev_update_time;

/*******************************************************************************
* ROS Parameter
*******************************************************************************/
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char imu_frame_id[30];
char mag_frame_id[30];

char joint_state_header_frame_id[30];

/*******************************************************************************
* Subscriber
*******************************************************************************/
// ros::Subscriber cmd_vel_sub;

/* from opencr */
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

/* from opencr */
// ros::Subscriber<std_msgs::Empty> reset_sub("reset", resetCallback);

/*******************************************************************************
* Publisher
*******************************************************************************/
// ros::Publisher joint_states_pub;
// ros::Publisher odom_pub;

// Joint(Dynamixel) state of Turtlebot3
/* from opencr */
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

// Odometry of Turtlebot3
/* from opencr */
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

// Version information of Turtlebot3
/* from opencr */
turtlebot3_msgs::VersionInfo version_info_msg;
ros::Publisher version_info_pub("firmware_version", &version_info_msg);

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// tf::TransformBroadcaster tf_broadcaster;

// TF of Turtlebot3
/* from opencr */
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

/*******************************************************************************
* Simulation Parameters
*******************************************************************************/
double wheel_speed_cmd[WHEEL_NUM] = {0.0, 0.0};
double goal_linear_velocity  = 0.0;
double goal_angular_velocity = 0.0;
double cmd_vel_timeout       = 1.0;

float  odom_pose[3];
float  odom_vel[3];

/*
???:  this doesn't appear to be used.  
pcov was in the .cpp file, and I moved it below.
*/
// double pose_cov[36];

/*
???:  why is this necessary?
used to set node params, which i removed.
push_back'ed into joint_states.name in .cpp
*/
std::string joint_states_name[2];

double last_position[WHEEL_NUM] = {0.0, 0.0};
double last_velocity[WHEEL_NUM] = {0.0, 0.0};

// assume TurtleBot3 Burger
double wheel_separation       = 0.287;
double turning_radius         = 0.1435;
double robot_radius           = 0.220;

/*
???:  this is not performed in opencr

moved the memcpy lines to the .cpp file
*/
double pcov[36] = { 0.1,   0,   0,   0,   0, 0,
                      0, 0.1,   0,   0,   0, 0,
                      0,   0, 1e6,   0,   0, 0,
                      0,   0,   0, 1e6,   0, 0,
                      0,   0,   0,   0, 1e6, 0,
                      0,   0,   0,   0,   0, 0.2};

/* The goal was to take this simulation (turtlebot3_fake) and map as much of the
opencr code (turtlebot3_core) to the header and cpp files so it can be compiled
using rosserial.

As a result, I removed the class definition and tried to make the file structure
match that of opencr turtlebot3_core.h.  This included a few instances of changing
arguments, presumably because the rosserial use of a function is different.

E.g., commandVelocityCallback receives TwistConstPtr cmd_vel_msg in turtlebot3_fake
and it receives Twist& cmd_vel_msg in turtlebot3_core.
*/

#endif // TURTLEBOT3_FAKE_ROSSERIAL_H_