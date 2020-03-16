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
 * Types of changes to map turtlebot3_core.cpp to this file:
 * - assume the turtlebot model is 'burger'
 * - remove treatment of Turtlebot3Fake as a class
 * - remove all the Turtlebot3Fake:: scoping
*******************************************************************************/

// #include <turtlebot3_fake/turtlebot3_fake.h>
#include "turtlebot3_rosserial.h"
#include "simulation_parameters.h"

/*******************************************************************************
* Global variables
*******************************************************************************/

// ROS Time
ros::Time last_cmd_vel_time;
ros::Time prev_update_time;

// Joint(Dynamixel) state of Turtlebot3
sensor_msgs::JointState joint_states;

// Odometry of Turtlebot3
nav_msgs::Odometry odom;

// Version information of Turtlebot3
turtlebot3_msgs::VersionInfo version_info_msg;

// TF of Turtlebot3
geometry_msgs::TransformStamped odom_tf;

/*******************************************************************************
* [New] Init function
*******************************************************************************/
bool initPhysical()
{
  // Setting for SLAM and navigation (odometry, joint states, TF)
  initOdom();

  initJointStates();

  prev_update_time = rosNow();

  return true;
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  last_cmd_vel_time = rosNow();

  goal_linear_velocity  = cmd_vel_msg.linear.x;
  goal_angular_velocity = cmd_vel_msg.angular.z;

  wheel_speed_cmd[LEFT]  = goal_linear_velocity - (goal_angular_velocity * wheel_separation / 2);
  wheel_speed_cmd[RIGHT] = goal_linear_velocity + (goal_angular_velocity * wheel_separation / 2);
}

/*******************************************************************************
* Initialization odometry data
*******************************************************************************/
void initOdom(void)
{
  // moved from the original init function.  pcov is in the header file
  memcpy(&(odom.pose.covariance),pcov,sizeof(double)*36);
  memcpy(&(odom.twist.covariance),pcov,sizeof(double)*36);

  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool updateOdometry(ros::Duration diff_time)
{
  double wheel_l, wheel_r; // rotation value of wheel [rad]
  double delta_s, delta_theta;
  double v[2], w[2];

  wheel_l = wheel_r     = 0.0;
  delta_s = delta_theta = 0.0;

  v[LEFT]  = wheel_speed_cmd[LEFT];
  w[LEFT]  = v[LEFT] / WHEEL_RADIUS;  // w = v / r
  v[RIGHT] = wheel_speed_cmd[RIGHT];
  w[RIGHT] = v[RIGHT] / WHEEL_RADIUS;

  last_velocity[LEFT]  = w[LEFT];
  last_velocity[RIGHT] = w[RIGHT];

  wheel_l = w[LEFT]  * diff_time.toSec();
  wheel_r = w[RIGHT] * diff_time.toSec();

  if(isnan(wheel_l))
  {
    wheel_l = 0.0;
  }

  if(isnan(wheel_r))
  {
    wheel_r = 0.0;
  }

  last_position[LEFT]  += wheel_l;
  last_position[RIGHT] += wheel_r;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  delta_theta = WHEEL_RADIUS * (wheel_r - wheel_l) / wheel_separation;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity
  odom_vel[0] = delta_s / diff_time.toSec();     // v
  odom_vel[1] = 0.0;
  odom_vel[2] = delta_theta / diff_time.toSec(); // w

  // i think these are necessary for robot_state_publisher and rviz
  odom.header.frame_id = getOdomHeaderFrameId();
  odom.child_frame_id  = getOdomChildFrameId();

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  // We should update the twist of the odometry
  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];

  return true;
}

/*******************************************************************************
* Initialization joint states data
*******************************************************************************/
void initJointStates(void)
{
  static char *joint_states_name[] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};

//   joint_states.header.frame_id = joint_state_header_frame_id;  // opencr.  this never gets used
  joint_states.name            = joint_states_name;

  joint_states.name_length     = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  joint_states.effort_length   = 0;  // possibly used by opencr? set to 0 or else serialisation crashes
}

/*******************************************************************************
* Update the joint states 
*******************************************************************************/
void updateJointStates(void)
{
  static double joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
  static double joint_states_vel[WHEEL_NUM] = {0.0, 0.0};

  joint_states_pos[LEFT]  = last_position[LEFT];
  joint_states_pos[RIGHT] = last_position[RIGHT];

  joint_states_vel[LEFT]  = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}

/*******************************************************************************
* Calculate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation = odom.pose.pose.orientation;
}

/*******************************************************************************
* Update function
*******************************************************************************/
bool updatePhysical()
{
  ros::Time time_now = rosNow();

  // i don't really understand what this is for, but seems necessary for robot_state_publisher and rviz
  // updateTFPrefix(nh.connected());

  // this is a bit odd, but in rosserial Time and Duration don't have a - operator
  // so i have to convert a Time to seconds (a double), subtract, then convert to a Duration
  ros::Duration step_time;
  step_time.fromSec(time_now.toSec() - prev_update_time.toSec());
  prev_update_time = time_now;

  // zero-ing after timeout
  if((time_now.toSec() - last_cmd_vel_time.toSec()) > cmd_vel_timeout)
  {
    wheel_speed_cmd[LEFT]  = 0.0;
    wheel_speed_cmd[RIGHT] = 0.0;
  }

  // odom
  updateOdometry(step_time);
  odom.header.stamp = time_now;
  // odom_pub.publish(&odom);

  // joint_states
  updateJointStates();
  joint_states.header.stamp = time_now;
  // joint_states_pub.publish(&joint_states);

  // tf
  updateTF(odom_tf);
  // tf_broadcaster.sendTransform(odom_tf);

  // sendLogMsg();

  return true;
}

/*******************************************************************************
* Access functions
*******************************************************************************/
// Joint(Dynamixel) state of Turtlebot3
sensor_msgs::JointState* getJointStates(void)
{
  return &joint_states;
}

// Odometry of Turtlebot3
nav_msgs::Odometry* getOdom(void)
{
  return &odom;
}

// Version information of Turtlebot3
turtlebot3_msgs::VersionInfo* getVersionInfoMsg(void)
{
  return &version_info_msg;
}

// TF of Turtlebot3
// unlike the other publishers, tf broadcaster takes a reference, not a pointer, as input, I think.
geometry_msgs::TransformStamped& getOdomTf(void)
{
  return odom_tf;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{

  std::string cl_svr_ip="128.232.65.230";
  std::string host_ip;

  if(argc==1)
  {
    host_ip = cl_svr_ip;
  }
  else if(argc==2)
  {
    host_ip=argv[1];
  }
  else
  {
    printf("Usage: turtlebot3_rosserial [host_ip]\n");
    return 1;
  }

  int count = 0;
  int spin_result;

  initPhysical();
  initComms(host_ip);

  while (1)
  {
    updatePhysical();
    updateComms();

    spin_result=spinComms();

    if(count%10 == 0)
    {
      printf("Loop count: %d\n", count);
    }
    count++;

    sleep(loop_rate);
  }

  return 0;
}
