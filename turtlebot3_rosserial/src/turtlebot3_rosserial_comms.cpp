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
 * Modified by Michael Dodson, 2020
*******************************************************************************/

#include "turtlebot3_rosserial.h"

/*******************************************************************************
* Global variables
*******************************************************************************/

// ROS NodeHandle
ros::NodeHandle nh;

// ROS Parameters
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char joint_state_header_frame_id[30];

// Publishers
ros::Publisher odom_pub("odom", getOdom());
ros::Publisher version_info_pub("firmware_version", getVersionInfoMsg());
ros::Publisher joint_states_pub("joint_states", getJointStates());
tf::TransformBroadcaster tf_broadcaster;

// Subscribers
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

/*******************************************************************************
* [New] Init function
*******************************************************************************/
bool initComms(std::string host_ip)
{
  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode((char *)host_ip.c_str());

  nh.subscribe(cmd_vel_sub);

  nh.advertise(version_info_pub);

  nh.advertise(odom_pub);

  nh.advertise(joint_states_pub);

  tf_broadcaster.init(nh);

  return true;
}

/*******************************************************************************
* Update function
*******************************************************************************/
bool updateComms(void)
{
  ros::Time time_now = rosNow();

  // i don't really understand what this is for, but seems necessary for robot_state_publisher and rviz
  updateTFPrefix(nh.connected());

  // odom
  odom_pub.publish(getOdom());

  // joint_states
  joint_states_pub.publish(getJointStates());

  // tf
  tf_broadcaster.sendTransform(getOdomTf());

  sendLogMsg();

  return true;
}

/*******************************************************************************
* Spin function
*******************************************************************************/
int spinComms(void)
{
    int spin_result;

    spin_result=nh.spinOnce();

    return spin_result;
}

/*******************************************************************************
* Publish msgs (version info)
*******************************************************************************/
void publishVersionInfoMsg(void)
{
  turtlebot3_msgs::VersionInfo* version_info_msg = getVersionInfoMsg();

  version_info_msg->hardware = "0.0.0";
  version_info_msg->software = "0.0.0";
  version_info_msg->firmware = FIRMWARE_VER;

  version_info_pub.publish(getVersionInfoMsg());
}

/*******************************************************************************
* Send log message
*******************************************************************************/
void sendLogMsg(void)
{
  static bool log_flag = false;
  char log_msg[100];  

  std::string name             = NAME;
  std::string firmware_version = FIRMWARE_VER;
  std::string git_branch       = GIT_BRANCH;
  std::string name_data        = "TB3:\t" + name;
  std::string fw_data          = "FW:\t\t" + firmware_version;
  std::string git_data         = "Git branch:\t" + git_branch;
   
  // const char* init_log_data = bringup_log.c_str();

  if (nh.connected())
  {
    if (log_flag == false)
    {      
      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      sprintf(log_msg, "Connected to Simulation!");
      nh.loginfo(log_msg);

      sprintf(log_msg, "%s", name_data.c_str());
      nh.loginfo(log_msg);

      sprintf(log_msg, "%s", fw_data.c_str());
      nh.loginfo(log_msg);

      sprintf(log_msg, "%s", git_data.c_str());
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
* Update TF Prefix
*******************************************************************************/
void updateTFPrefix(bool isConnected)
{
  static bool isChecked = false;
  char log_msg[50];

  if (isConnected)
  {
    if (isChecked == false)
    {
      nh.getParam("~tf_prefix", &get_tf_prefix);

      if (!strcmp(get_tf_prefix, ""))
      {
        sprintf(odom_header_frame_id, "odom");
        sprintf(odom_child_frame_id, "base_footprint");  

        sprintf(joint_state_header_frame_id, "base_link");
      }
      else
      {
        strcpy(odom_header_frame_id, get_tf_prefix);
        strcpy(odom_child_frame_id, get_tf_prefix);

        strcpy(joint_state_header_frame_id, get_tf_prefix);

        strcat(odom_header_frame_id, "/odom");
        strcat(odom_child_frame_id, "/base_footprint");

        strcat(joint_state_header_frame_id, "/base_link");
      }

      sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
      nh.loginfo(log_msg); 

      isChecked = true;
    }
  }
  else
  {
    isChecked = false;
  }
}

/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now();
}

/*******************************************************************************
* Access functions
*******************************************************************************/
char* getOdomHeaderFrameId(void)
{
  return odom_header_frame_id;
}

char* getOdomChildFrameId(void)
{
  return odom_child_frame_id;
}