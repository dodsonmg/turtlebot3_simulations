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
#include "turtlebot3_fake.h"

// Turtlebot3Fake::Turtlebot3Fake()
// : nh_priv_("~")
// {
//   //Init fake turtlebot node
//   bool init_result = init();
//   ROS_ASSERT(init_result);
// }

// Turtlebot3Fake::~Turtlebot3Fake()
// {
// }

/*******************************************************************************
* Init function
*******************************************************************************/
// bool init()
// {
  // initialize ROS parameter

  /* 
  assume we're dealing with burger.  
  moved definitions into .h file
  */
  // std::string robot_model = nh_.param<std::string>("tb3_model", "");

  // if (!robot_model.compare("burger"))
  // {
  //   wheel_seperation_ = 0.160;
  //   turning_radius_   = 0.080;
  //   robot_radius_     = 0.105;
  // }
  // else if (!robot_model.compare("waffle") || !robot_model.compare("waffle_pi"))
  // {
  //   wheel_seperation_ = 0.287;
  //   turning_radius_   = 0.1435;
  //   robot_radius_     = 0.220;
  // }

/* ???: what are these parameter definitons used for? */
  // nh_.param("wheel_left_joint_name", joint_states_name_[LEFT],  std::string("wheel_left_joint"));
  // nh_.param("wheel_right_joint_name", joint_states_name_[RIGHT],  std::string("wheel_right_joint"));
  // nh_.param("joint_states_frame", joint_states_.header.frame_id, std::string("base_footprint"));
  // nh_.param("odom_frame", odom_.header.frame_id, std::string("odom"));
  // nh_.param("base_frame", odom_.child_frame_id, std::string("base_footprint"));

  /* 
  moved initialisations into .h file
  */
  // initialize variables
  // wheel_speed_cmd_[LEFT]  = 0.0;
  // wheel_speed_cmd_[RIGHT] = 0.0;
  // goal_linear_velocity_   = 0.0;
  // goal_angular_velocity_  = 0.0;
  // cmd_vel_timeout_        = 1.0;
  // last_position_[LEFT]    = 0.0;
  // last_position_[RIGHT]   = 0.0;
  // last_velocity_[LEFT]    = 0.0;
  // last_velocity_[RIGHT]   = 0.0;

  // double pcov[36] = { 0.1,   0,   0,   0,   0, 0,
  //                       0, 0.1,   0,   0,   0, 0,
  //                       0,   0, 1e6,   0,   0, 0,
  //                       0,   0,   0, 1e6,   0, 0,
  //                       0,   0,   0,   0, 1e6, 0,
  //                       0,   0,   0,   0,   0, 0.2};
  // memcpy(&(odom_.pose.covariance),pcov,sizeof(double)*36);
  // memcpy(&(odom_.twist.covariance),pcov,sizeof(double)*36);

  // odom_pose_[0] = 0.0;
  // odom_pose_[1] = 0.0;
  // odom_pose_[2] = 0.0;

  // odom_vel_[0] = 0.0;
  // odom_vel_[1] = 0.0;
  // odom_vel_[2] = 0.0;

  /*
  These are now initialised in initJointStates(), copied from opencr
  */
  // joint_states_.name.push_back(joint_states_name_[LEFT]);
  // joint_states_.name.push_back(joint_states_name_[RIGHT]);
  // joint_states_.position.resize(2,0.0);
  // joint_states_.velocity.resize(2,0.0);
  // joint_states_.effort.resize(2,0.0);

  /*
  This is now accomplished in the header file.
  */
  // initialize publishers
  // joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
  // odom_pub_         = nh_.advertise<nav_msgs::Odometry>("odom", 100);

  /*
  This is now accomplished in the header file.
  */
  // initialize subscribers
  // cmd_vel_sub_  = nh_.subscribe("cmd_vel", 100,  &commandVelocityCallback, this);

  /*
  moved to new init() function
  */
  // prev_update_time_ = ros::Time::now();

  // return true;
// }

/*******************************************************************************
* [New] Init function
*
* mostly copied from opencr setup() function
*******************************************************************************/

bool init()
{
  // DEBUG_SERIAL.begin(57600);  // not sure how this is used

  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);

  nh.subscribe(cmd_vel_sub);
  // nh.subscribe(sound_sub);
  // nh.subscribe(motor_power_sub);
  nh.subscribe(reset_sub);

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
  prev_update_time_ = rosNow();

  // pinMode(LED_WORKING_CHECK, OUTPUT);

  // setup_end = true;

  return true;
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::TwistConstPtr cmd_vel_msg)
{
  last_cmd_vel_time_ = ros::Time::now();

  goal_linear_velocity_  = cmd_vel_msg->linear.x;
  goal_angular_velocity_ = cmd_vel_msg->angular.z;

  wheel_speed_cmd_[LEFT]  = goal_linear_velocity_ - (goal_angular_velocity_ * wheel_seperation_ / 2);
  wheel_speed_cmd_[RIGHT] = goal_linear_velocity_ + (goal_angular_velocity_ * wheel_seperation_ / 2);
}

/*******************************************************************************
* Publish msgs (version info)
*
* copied from opencr
*******************************************************************************/
void publishVersionInfoMsg(void)
{
  version_info_msg_.hardware = "0.0.0";
  version_info_msg_.software = "0.0.0";
  version_info_msg_.firmware = FIRMWARE_VER;

  version_info_pub.publish(&version_info_msg_);
}

/*******************************************************************************
* Initialization odometry data
*
* copied from opencr
*******************************************************************************/
void initOdom(void)
{
  init_encoder = true;

  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }

  odom_.pose.pose.position.x = 0.0;
  odom_.pose.pose.position.y = 0.0;
  odom_.pose.pose.position.z = 0.0;

  odom_.pose.pose.orientation.x = 0.0;
  odom_.pose.pose.orientation.y = 0.0;
  odom_.pose.pose.orientation.z = 0.0;
  odom_.pose.pose.orientation.w = 0.0;

  odom_.twist.twist.linear.x  = 0.0;
  odom_.twist.twist.angular.z = 0.0;
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

  v[LEFT]  = wheel_speed_cmd_[LEFT];
  w[LEFT]  = v[LEFT] / WHEEL_RADIUS;  // w = v / r
  v[RIGHT] = wheel_speed_cmd_[RIGHT];
  w[RIGHT] = v[RIGHT] / WHEEL_RADIUS;

  last_velocity_[LEFT]  = w[LEFT];
  last_velocity_[RIGHT] = w[RIGHT];

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

  last_position_[LEFT]  += wheel_l;
  last_position_[RIGHT] += wheel_r;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  delta_theta = WHEEL_RADIUS * (wheel_r - wheel_l) / wheel_seperation_;

  // compute odometric pose
  odom_pose_[0] += delta_s * cos(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[1] += delta_s * sin(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[2] += delta_theta;

  // compute odometric instantaneouse velocity
  odom_vel_[0] = delta_s / diff_time.toSec();     // v
  odom_vel_[1] = 0.0;
  odom_vel_[2] = delta_theta / diff_time.toSec(); // w

  odom_.pose.pose.position.x = odom_pose_[0];
  odom_.pose.pose.position.y = odom_pose_[1];
  odom_.pose.pose.position.z = 0;
  odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose_[2]);

  // We should update the twist of the odometry
  odom_.twist.twist.linear.x  = odom_vel_[0];
  odom_.twist.twist.angular.z = odom_vel_[2];

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

  joint_states.header.frame_id = joint_state_header_frame_id;
  joint_states.name            = joint_states_name;

  joint_states.name_length     = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  joint_states.effort_length   = WHEEL_NUM;
}

/*******************************************************************************
* Calculate the joint states
*******************************************************************************/
void updateJoint(void)
{
  joint_states_.position[LEFT]  = last_position_[LEFT];
  joint_states_.position[RIGHT] = last_position_[RIGHT];
  joint_states_.velocity[LEFT]  = last_velocity_[LEFT];
  joint_states_.velocity[RIGHT] = last_velocity_[RIGHT];
}

/*******************************************************************************
* Calculate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom_.header;
  odom_tf.child_frame_id = odom_.child_frame_id;
  odom_tf.transform.translation.x = odom_.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_.pose.pose.position.z;
  odom_tf.transform.rotation = odom_.pose.pose.orientation;
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

  String name             = NAME;
  String firmware_version = FIRMWARE_VER;
  String bringup_log      = "This core(v" + firmware_version + ") is compatible with TB3 " + name;
   
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
  ros::Duration step_time = time_now - prev_update_time_;
  prev_update_time_ = time_now;

  // zero-ing after timeout
  if((time_now - last_cmd_vel_time_).toSec() > cmd_vel_timeout_)
  {
    wheel_speed_cmd_[LEFT]  = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
  }

  // odom
  updateOdometry(step_time);
  odom_.header.stamp = time_now;
  odom_pub_.publish(odom_);

  // joint_states
  updateJoint();
  joint_states_.header.stamp = time_now;
  joint_states_pub_.publish(joint_states_);

  // tf
  // geometry_msgs::TransformStamped odom_tf;  // moved to header file
  updateTF(odom_tf);
  tf_broadcaster_.sendTransform(odom_tf);

  sendLogMsg();

  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  // ros::init(argc, argv, "turtlebot3_fake_node");
  // Turtlebot3Fake tb3fake;
  init();

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    // tb3fake.update();
    update();
    // ros::spinOnce();
    nh.spinOnce();  // from opencr
    loop_rate.sleep();
  }

  return 0;
}