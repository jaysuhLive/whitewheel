#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <sstream>
#include "geometry_msgs/Twist.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "freeway_joyfw/stm_fw_msg.h"
#include "freeway_joyfw/stm_am_msg.h"
#include "freeway_joyfw/stm_fw_srv.h"

#define RP_LIDAR_S2 true

class Freeway_Joy_Fw
{
  public:
    Freeway_Joy_Fw(ros::NodeHandle *n)
    {
      cmd_pub = n->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
      am_mode_pub = n->advertise<freeway_joyfw::stm_am_msg>("freeway/am_status", 10);
      move_base_flex_cancel_pub = n->advertise<actionlib_msgs::GoalID>("move_base_flex/move_base/cancel", 10);
      diag_sub = n->subscribe("freeway/diagnostics", 100, &Freeway_Joy_Fw::get_diagnostics_cb, this);
      front_obstacle_sub = n->subscribe("/freeway/front_obstacle", 100, &Freeway_Joy_Fw::front_obstacle_cb, this);
      cmd_vel_ui_sub = n->subscribe("/cmd_vel_ui", 10, &Freeway_Joy_Fw::cmd_vel_ui_cb, this);
      diagnostics_time;
      stm_msg;
      front_obstacle_detected = false;
    }

    bool am_mode_cb(freeway_joyfw::stm_fw_srv::Request &req, freeway_joyfw::stm_fw_srv::Response &res);

    void front_obstacle_cb(const std_msgs::Bool& front_obstacle_msg)
    {
      front_obstacle_detected = front_obstacle_msg.data;
    }
    

    void get_diagnostics_cb(const freeway_joyfw::stm_fw_msg &diag_msg) {
      geometry_msgs::Twist cmd_vel_msg;
      actionlib_msgs::GoalID empty_goal;
      stm_msg = diag_msg;
      diagnostics_time = ros::Time::now();
      if (stm_msg.am_status == true && stm_msg.e_stop_status == true)
      {
        if(front_obstacle_detected) {
          if (diag_msg.cmd_vel_mcu.linear.x > 0.0) {
              cmd_vel_msg.linear.x = 0.0;
          }
          else {
            cmd_vel_msg.linear.x =  diag_msg.cmd_vel_mcu.linear.x;
          }

          cmd_vel_msg.angular.z = diag_msg.cmd_vel_mcu.angular.z;
          cmd_pub.publish(cmd_vel_msg);
          
        }
        else {
          cmd_vel_msg.linear.x = diag_msg.cmd_vel_mcu.linear.x;
          cmd_vel_msg.angular.z = diag_msg.cmd_vel_mcu.angular.z;
          cmd_pub.publish(cmd_vel_msg);
        }
      }
      else if (stm_msg.e_stop_status == false)
      {
        cmd_vel_msg.linear.x = 0.0;//diag_msg.cmd_vel_mcu.linear.x;
        cmd_vel_msg.angular.z = 0.0;//diag_msg.cmd_vel_mcu.angular.z;
        cmd_pub.publish(cmd_vel_msg);
        move_base_flex_cancel_pub.publish(empty_goal);
      }
    }

    void cmd_vel_ui_cb(const geometry_msgs::Twist &cmd_vel_ui_msg) {
    geometry_msgs::Twist cmd_vel_msg;
    ros::Duration timeout_duration(0.5);
    bool active_flag = false;
    if (ros::Time::now() - diagnostics_time < timeout_duration) active_flag = true;
    else active_flag = false;

    if (active_flag == false)
    {
        if(front_obstacle_detected) {
          if (cmd_vel_ui_msg.linear.x > 0.0) {
              cmd_vel_msg.linear.x = 0.0;
          }
          else {
            cmd_vel_msg.linear.x =  cmd_vel_ui_msg.linear.x;
          }

          cmd_vel_msg.angular.z = cmd_vel_ui_msg.angular.z;
          cmd_pub.publish(cmd_vel_msg);

        }
        else {
          cmd_vel_msg.linear.x = cmd_vel_ui_msg.linear.x;
          cmd_vel_msg.angular.z = cmd_vel_ui_msg.angular.z;
          cmd_pub.publish(cmd_vel_msg);
        }
   }
      if (active_flag == true && stm_msg.e_stop_status == false)
      {
        cmd_vel_msg.linear.x = 0.0;//diag_msg.cmd_vel_mcu.linear.x;
        cmd_vel_msg.angular.z = 0.0;//diag_msg.cmd_vel_mcu.angular.z;
        cmd_pub.publish(cmd_vel_msg);
      }
    }

  private:
    ros::Publisher cmd_pub;
    ros::Publisher move_base_flex_cancel_pub;
    ros::Publisher am_mode_pub;
    ros::Subscriber diag_sub;
    ros::Subscriber front_obstacle_sub;
    ros::Subscriber cmd_vel_ui_sub;
    freeway_joyfw::stm_fw_msg stm_msg;
    ros::Time diagnostics_time;
    bool front_obstacle_detected = false;
};

bool Freeway_Joy_Fw::am_mode_cb(freeway_joyfw::stm_fw_srv::Request &req, freeway_joyfw::stm_fw_srv::Response &res)
{
  freeway_joyfw::stm_am_msg am_msg;
  res.result = req.am_mode;
  am_msg.am_status2=res.result;
  am_mode_pub.publish(am_msg);
  ROS_INFO("res.result : %d", res.result);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "freeway_joy_node");
  ros::NodeHandle n;

  Freeway_Joy_Fw Fj = Freeway_Joy_Fw(&n);
  ros::ServiceServer service = n.advertiseService("am_mode", &Freeway_Joy_Fw::am_mode_cb, &Fj);
  
  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
