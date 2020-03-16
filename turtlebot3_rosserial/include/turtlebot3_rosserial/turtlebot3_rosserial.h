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
#include <unistd.h> // supports sleep function

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

/*******************************************************************************
* TurtleBot3 Burger Parameters
*******************************************************************************/
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
/* from opencr */
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);

/*******************************************************************************
* Function prototypes
*******************************************************************************/
bool initPhysical(void);
bool initComms(std::string host_ip);

bool updatePhysical(void);
bool updateComms(void);

int spinComms(void);
void sendLogMsg(void);

void initOdom(void);  //copied from opencr
void initJointStates(void);  // copied from opencr

bool updateOdometry(ros::Duration diff_time);  // arguments are void in opencr
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateTFPrefix(bool isConnected);
void updateJointStates(void);

void publishVersionInfoMsg(void);  // copied from opencr

ros::Time rosNow(void);  // from opencr

char* getOdomHeaderFrameId(void);
char* getOdomChildFrameId(void);

sensor_msgs::JointState* getJointStates(void);
nav_msgs::Odometry* getOdom(void);
turtlebot3_msgs::VersionInfo* getVersionInfoMsg(void);
geometry_msgs::TransformStamped& getOdomTf(void);

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
