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
#include "turtlebot3_fake_rosserial.h"

/*******************************************************************************
* [New] Init function
*
* mostly copied from opencr setup() function
*******************************************************************************/

bool init()
{
  // DEBUG_SERIAL.begin(57600);  // not sure how this is used

  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode("128.232.65.230");
  nh.getHardware()->setBaud(115200);

  nh.subscribe(cmd_vel_sub);
  // nh.subscribe(sound_sub);
  // nh.subscribe(motor_power_sub);
//   nh.subscribe(reset_sub);

  // nh.advertise(sensor_state_pub);  
  nh.advertise(version_info_pub);
  // nh.advertise(imu_pub);
  // nh.advertise(cmd_vel_rc100_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  // nh.advertise(battery_state_pub);
  // nh.advertise(mag_pub);

  tf_broadcaster.init(nh);

  // Setting for Dynamixel motors
  // motor_driver.init(NAME);

  // Setting for IMU
  // sensors.init();

  // Init diagnosis
  // diagnosis.init();

  // Setting for ROBOTIS RC100 remote controller and cmd_vel
  // controllers.init(MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY);

  // Setting for SLAM and navigation (odometry, joint states, TF)
  initOdom();

  initJointStates();

  // prev_update_time = millis();  // this returns ticks since the board started
  prev_update_time = rosNow();

  // pinMode(LED_WORKING_CHECK, OUTPUT);

  // setup_end = true;

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
* Publish msgs (version info)
*
* copied from opencr
*******************************************************************************/
void publishVersionInfoMsg(void)
{
  version_info_msg.hardware = "0.0.0";
  version_info_msg.software = "0.0.0";
  version_info_msg.firmware = FIRMWARE_VER;

  version_info_pub.publish(&version_info_msg);
}

/*******************************************************************************
* Initialization odometry data
*
* copied from opencr
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
*
* copied from opencr
*******************************************************************************/
void initJointStates(void)
{
  static char *joint_states_name[] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};

//   joint_states.header.frame_id = joint_state_header_frame_id;  // opencr.  this never gets set
  joint_states.name            = joint_states_name;

  joint_states.name_length     = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  joint_states.effort_length   = WHEEL_NUM;
}

/*******************************************************************************
* Calculate the joint states
*******************************************************************************/
// void updateJoint(void)
// {
//   joint_states.position[LEFT]  = last_position[LEFT];
//   joint_states.position[RIGHT] = last_position[RIGHT];
//   joint_states.velocity[LEFT]  = last_velocity[LEFT];
//   joint_states.velocity[RIGHT] = last_velocity[RIGHT];
// }

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
* Send log message
*
* copied from opencr
*******************************************************************************/
void sendLogMsg(void)
{
  static bool log_flag = false;
  char log_msg[100];  

  std::string name             = NAME;
  std::string firmware_version = FIRMWARE_VER;
  std::string bringup_log      = "This core(v" + firmware_version + ") is compatible with TB3 " + name;
   
  const char* init_log_data = bringup_log.c_str();

  if (nh.connected())
  {
    if (log_flag == false)
    {      
      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      sprintf(log_msg, "Connected to Simulation!");
      nh.loginfo(log_msg);

      sprintf(log_msg, init_log_data);
      nh.loginfo(log_msg);

      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      log_flag = true;
    }
  }
  else
  {
    log_flag = false;
  }
}

/*******************************************************************************
* ros::Time::now() implementation
*
* copied from opencr
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now();
}

/*******************************************************************************
* Time Interpolation function (deprecated)
*
* copied from opencr
* not sure if this is used...
*******************************************************************************/
ros::Time addMicros(ros::Time & t, uint32_t _micros)
{
  uint32_t sec, nsec;

  sec  = _micros / 1000 + t.sec;
  nsec = _micros % 1000000000 + t.nsec;

  return ros::Time(sec, nsec);
}

/*******************************************************************************
* Update function
*******************************************************************************/
bool update()
{
  ros::Time time_now = rosNow();

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
  odom_pub.publish(&odom);

  // joint_states
  updateJointStates();
  joint_states.header.stamp = time_now;
  joint_states_pub.publish(&joint_states);

  // tf
  // geometry_msgs::TransformStamped odom_tf;  // moved to header file
  updateTF(odom_tf);
  tf_broadcaster.sendTransform(odom_tf);

  sendLogMsg();

  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{

  int count = 0;
  // ros::init(argc, argv, "turtlebot3_fake_node");
  // Turtlebot3Fake tb3fake;
  init();

  while (1)
  {
    // tb3fake.update();
    update();
    // ros::spinOnce();
    nh.spinOnce();  // from opencr
    printf("in the loop [%d]\n", count);
    count++;
    sleep(1);
  }

  return 0;
}
