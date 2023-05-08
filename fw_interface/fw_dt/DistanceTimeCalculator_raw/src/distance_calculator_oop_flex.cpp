#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <string>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"
#include "mbf_msgs/MoveBaseActionFeedback.h"
#include "freeway_msgs/DistanceTimeCalculator.h"
#include "freeway_msgs/FreewayStatus.h"

#define fix_pose true
#define fix_pose_pub false
#define POSE_RESET_TIME 15.0

// void get_goal_cb(const geometry_msgs::PoseStamped &goal_msg) {
//   geometry_msgs::PoseStamped goal;
//   goal.header.stamp = ros::Time::now();
//   goal.header.frame_id = "/map";
//   goal.pose.position.x = goal_msg.pose.position.x;
//   goal.pose.position.y = goal_msg.pose.position.y;
//   goal.pose.orientation.z = goal_msg.pose.orientation.z;
//   goal.pose.orientation.w = goal_msg.pose.orientation.w;
// }

class Distance_TimeCalculator
{
  public:
    Distance_TimeCalculator(ros::NodeHandle *n)
    {
      resume_pub = n->advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 100);
      resume_pub2 = n->advertise<std_msgs::Empty>("freeway/resume", 10);
      cancel_pub = n->advertise<actionlib_msgs::GoalID>("move_base_flex/move_base/cancel", 10);
      cancel_pub2 = n->advertise<std_msgs::Empty>("freeway/goal_cancel", 10);
      cmd_pub = n->advertise<geometry_msgs::Twist>("cmd_vel/nav", 10);
      pose_pub = n->advertise<geometry_msgs::PoseWithCovarianceStamped>("freeway/initialpose",10);
      cancel_sub = n->subscribe("freeway/goal_cancel", 100, &Distance_TimeCalculator::cancel_cb, this);
      feedback_sub = n->subscribe("move_base_flex/move_base/feedback", 1000, &Distance_TimeCalculator::get_feedback_cb, this);
      status_sub = n->subscribe("move_base_flex/move_base/status", 10, &Distance_TimeCalculator::get_status_cb, this);
      velocity_sub = n->subscribe("cmd_vel", 10, &Distance_TimeCalculator::get_velocity_cb, this);
      getpath_sub = n->subscribe("move_base_flex/TebLocalPlannerROS/global_plan", 10, &Distance_TimeCalculator::get_globalpath_cb, this);
      resume_sub = n->subscribe("freeway/resume",10, &Distance_TimeCalculator::resume_cb, this);
      goal_sub = n->subscribe("move_base_simple/goal", 10, &Distance_TimeCalculator::goal_cb, this);
      pose_sub = n->subscribe("freeway/localization_pose", 10, &Distance_TimeCalculator::pose_cb, this);
      moving_check_sub = n->subscribe("freeway/moving_check", 10 , &Distance_TimeCalculator::moving_check_cb ,this);
      finitialpose_sub = n->subscribe("freeway/finitialpose", 10, &Distance_TimeCalculator::finitialpose_cb, this);
      front_blocked_path_sub = n->subscribe("/freeway/front_blocked_path", 10, &Distance_TimeCalculator::front_blocked_path_cb, this);
      clearcostmap_pub = n->serviceClient<std_srvs::Empty>("move_base_flex/clear_costmaps");

      total_vel = 0.0;
      status_flag=0;
      first_time=1; 
      status_info_=0;
      current_robot_pose;
      prev_current_robot_pose;
      final_goal;
      final_pose;
      finitial_pose;
      finitial_pose_flag = false;
      final_pose_time;
      final_front_blocked_path_msg_time;
      move_base_GlobalPlanner_plan_Time=0.0;
      first_global_path_distance=0.0;
      global_path_percentage=0.0;
      remaining_time=0.0;
      msg_global_path_distance=0.0;
      traveled_distance=0.0;
      average_total_vel=0.0;
      remaining_percentage=0.0;
      global_path_flag = false;
      moving_check_flag = false;
      front_blocked_path_flag = false;
    }

void cancel_cb(const std_msgs::Empty &cancel_msg) {
  geometry_msgs::Twist cmd_vel_msg;
  actionlib_msgs::GoalID empty_goal;
  cmd_vel_msg.linear.x = 0.0;
  cmd_vel_msg.angular.z = 0.0;

  cmd_pub.publish(cmd_vel_msg);
  cancel_pub.publish(empty_goal);
}

void resume_cb(const std_msgs::Empty &resume_msg) {
    ROS_INFO("status_info_: %d\n", status_info_);
    ROS_INFO("Resume Triggered to  x:%f, y:%f, z:%f", final_goal.pose.position.x, final_goal.pose.position.y, final_goal.pose.position.z);
    if((!(status_info_ == 4 || status_info_ == 3)) && (!(final_goal.pose.position.x == 0.0 || final_goal.pose.position.y == 0.0))) {
      resume_pub.publish(final_goal);
    }
}

void goal_cb(const geometry_msgs::PoseStamped &goal_msg) {
  final_goal = goal_msg;
}

void get_status_cb(const actionlib_msgs::GoalStatusArray &status_msg) {
  // status_flag = status_msg.status_list[0].status;
  int is=0;
  if (!status_msg.status_list.empty()) {
    for(size_t i=0; i<status_msg.status_list.size()-1; i++)
    {
      is++;
    }
      status_info_ = status_msg.status_list[is].status;
  }
  //else ROS_INFO("status_list array is empty");
  //ROS_INFO("status flag: %u", status_flag);
}

void get_velocity_cb(const geometry_msgs::Twist &cur_vel) {
  static uint counter_N1 = 1;
  float cur_x_vel = cur_vel.linear.x;
  float cur_y_vel = cur_vel.linear.y;
  float cur_az_vel = cur_vel.angular.z;

  total_vel = sqrt((cur_x_vel*cur_x_vel)+(cur_y_vel*cur_y_vel)+(cur_az_vel*cur_az_vel));
  // ROS_INFO("current velocity : %f", total_vel);
  
  if (status_flag==1) {
  average_total_vel = (float)(average_total_vel*(counter_N1-1)+total_vel)/counter_N1;
  //ROS_INFO("Average velocity : %f", average_total_vel);
  counter_N1++;
  }
  else { counter_N1 = 1; average_total_vel = 0.0; }
  //ROS_INFO("counter number : %u", counter_N1);
}

void get_globalpath_cb(const nav_msgs::Path &globalpath_msgs) {
 move_base_GlobalPlanner_plan_Time = ros::Time::now().toSec();
 static uint counter_N2=1;
 double current_a_x = 0.0;
 double current_b_x = 0.0;
 double current_a_y = 0.0;
 double current_b_y = 0.0;
 double global_path_distance=0.0;
 float pre_global_path_distance=0.0;

  if (status_flag==1){
    if ( !globalpath_msgs.poses.empty()) {
      for (size_t i=0; i < globalpath_msgs.poses.size()-1; i++)
      {
       current_a_x = globalpath_msgs.poses[i].pose.position.x;
       current_b_x = globalpath_msgs.poses[i+1].pose.position.x;
       current_a_y = globalpath_msgs.poses[i].pose.position.y;
       current_b_y = globalpath_msgs.poses[i+1].pose.position.y;
       global_path_distance += hypot((current_b_x - current_a_x), (current_b_y - current_a_y));
       msg_global_path_distance = global_path_distance;
       // ROS_INFO("hypot value : %lf", hypot((current_b_x - current_a_x), (current_b_y - current_a_y)));
       //global_path_distance += sqrt(pow(current_b_x - current_a_x,2) + pow(current_b_y - current_a_y,2));
       if(counter_N2==1) first_global_path_distance = global_path_distance;
       global_path_percentage = ((first_global_path_distance-global_path_distance)/first_global_path_distance)*100;
      }
    }
    remaining_time = global_path_distance/average_total_vel;
    //ROS_INFO("global_path_percentage : %f ", global_path_percentage);
    pre_global_path_distance = global_path_distance;
    // if (pre_global_path_distance - global_path_distance > 0) counter_N2=1;
    counter_N2++;

    global_path_flag = true;
  } 
  else { counter_N2=1; remaining_time = 0.0; global_path_distance=0.0; first_global_path_distance=0.0;}

  //ROS_INFO("remaining_time : %f",remaining_time);
  //ROS_INFO("global_path_distance : %f",msg_global_path_distance);
}

void get_feedback_cb(const mbf_msgs::MoveBaseActionFeedback &feedback_msgs){
  if (global_path_flag) {
  if(first_time==1){
    prev_current_robot_pose.position.x = feedback_msgs.feedback.current_pose.pose.position.x;
    prev_current_robot_pose.position.y = feedback_msgs.feedback.current_pose.pose.position.y;
    }
  else if (first_time!=1){
    current_robot_pose.position.x = feedback_msgs.feedback.current_pose.pose.position.x;
    current_robot_pose.position.y = feedback_msgs.feedback.current_pose.pose.position.y;
  }
  traveled_distance += hypot((current_robot_pose.position.x - prev_current_robot_pose.position.x), (prev_current_robot_pose.position.y - current_robot_pose.position.y)); 
  prev_current_robot_pose.position.x = current_robot_pose.position.x;
  prev_current_robot_pose.position.y = current_robot_pose.position.y;
  if(status_flag==1)first_time++;
  else if(status_flag !=1)first_time = 1;

  //ROS_INFO("robot_traveled_distance : %lf",traveled_distance);

  remaining_percentage = traveled_distance/(msg_global_path_distance+traveled_distance)*100;
  }
}

void check_update_time(ros::Time current_time)
{
  if ((current_time.toSec() - move_base_GlobalPlanner_plan_Time) < 2.0) { status_flag=1; }
  else { status_flag=0; remaining_time = 0.0; msg_global_path_distance=0.0; first_global_path_distance=0.0; traveled_distance=0.0; first_time=0; remaining_percentage=0.0; global_path_flag=false;}
}

void check_pose_time(ros::Time current_time, std::string file_path)
{
  if ((current_time.toSec() - final_pose_time.toSec()) > POSE_RESET_TIME && final_pose.header.seq !=0) { 
    std_srvs::Empty srv;
    if(clearcostmap_pub.call(srv)) ROS_INFO("clear costmap call sucessfully...");
    tf::Quaternion q(
	final_pose.pose.pose.orientation.x,
	final_pose.pose.pose.orientation.y,
	final_pose.pose.pose.orientation.z,
	final_pose.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    final_pose.header.seq=final_pose.header.seq+1;
    final_pose.header.stamp.sec=current_time.toSec();
    final_pose.header.stamp.nsec=current_time.toNSec();
    final_pose.pose.covariance[0]=0.05;
    final_pose.pose.covariance[1]=0.0;
    final_pose.pose.covariance[6]=0.0;
    final_pose.pose.covariance[7]=0.05;
    final_pose.pose.covariance[35]=0.02;
    if(fix_pose_pub) { 
	pose_pub.publish(final_pose);
     ROS_INFO("set initialpose to last pose");
    }
    final_pose_time = current_time;

    std::ofstream writeFile(file_path.data());
    if(writeFile.is_open()) {
      writeFile << "initial_pose_x: " << final_pose.pose.pose.position.x << std::endl;
      writeFile << "initial_pose_y: " << final_pose.pose.pose.position.y << std::endl;
      writeFile << "initial_pose_a: " << yaw<< std::endl;
      writeFile.close();
      ROS_INFO("amcl param update");
      }    
   }
}
void pose_cb(const geometry_msgs::PoseWithCovarianceStamped &pose_msgs) {
  if (!moving_check_flag || finitial_pose_flag) {
    final_pose_time = ros::Time::now();
    final_pose = pose_msgs;
    finitial_pose_flag = false;
  }
}

void check_block_time(ros::Time current_time) {
 if ((current_time.toSec() - final_front_blocked_path_msg_time.toSec()) <= 0.2 && front_blocked_path_flag == true) {
   std_msgs::Empty stop_msg;
   cancel_pub2.publish(stop_msg);
   front_blocked_path_flag = false; 
  }
 else if ((current_time.toSec() - final_front_blocked_path_msg_time.toSec()) > 0.2 && front_blocked_path_flag == false) {
    //std_msgs::Empty resume_msg;
    //resume_pub2.publish(resume_msg);
    front_blocked_path_flag = true;
  }
}

void front_blocked_path_cb(const std_msgs::Bool &front_blocked_path_msg) {
 final_front_blocked_path_msg_time = ros::Time::now();
 //front_blocked_path_data = true;
}

void moving_check_cb(const std_msgs::Bool &moving_check_msg) {
    if (moving_check_msg.data) moving_check_flag = true; //it's not moving
    else if (!moving_check_msg.data) moving_check_flag = false; //it's moving
}

void finitialpose_cb(const geometry_msgs::PoseWithCovarianceStamped &fpose_msgs) {
  finitial_pose = fpose_msgs;
  finitial_pose_flag = true;
  pose_pub.publish(finitial_pose);
}

double r_msg_global_path_distance()
{
  return msg_global_path_distance;
}

double r_remaining_time()
{
  return remaining_time;
}

double r_traveled_distance()
{
  return traveled_distance;
}

double r_remaining_percentage()
{
  return remaining_percentage;
}

double r_status_info_()
{
  return status_info_;
}

  private:
    ros::Publisher resume_pub;
    ros::Publisher resume_pub2;
    ros::Publisher cmd_pub;
    ros::Publisher cancel_pub;
    ros::Publisher cancel_pub2;
    ros::Publisher pose_pub;
    ros::Subscriber cancel_sub;
    ros::Subscriber feedback_sub;
    ros::Subscriber status_sub;
    ros::Subscriber velocity_sub; 
    ros::Subscriber getpath_sub;
    ros::Subscriber resume_sub;
    ros::Subscriber goal_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber moving_check_sub;
    ros::Subscriber finitialpose_sub;
    ros::Subscriber front_blocked_path_sub;
    ros::ServiceClient clearcostmap_pub;

    float total_vel;
    int status_flag;
    int first_time; 
    uint8_t status_info_;
    geometry_msgs::Pose current_robot_pose;
    geometry_msgs::Pose prev_current_robot_pose;
    geometry_msgs::PoseStamped final_goal;
    geometry_msgs::PoseWithCovarianceStamped final_pose;
    geometry_msgs::PoseWithCovarianceStamped finitial_pose;
    double move_base_GlobalPlanner_plan_Time;
    ros::Time final_pose_time;
    ros::Time final_front_blocked_path_msg_time;
    float first_global_path_distance;
    float global_path_percentage;
    float remaining_time;
    double msg_global_path_distance;
    double traveled_distance;
    float average_total_vel;
    double remaining_percentage;
    bool global_path_flag;
    bool moving_check_flag;
    bool finitial_pose_flag;
    bool front_blocked_path_flag;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance_calculator_oop_flex_node");
  ros::NodeHandle n;
  ros::Publisher distancetimecalculator_pub;
  freeway_msgs::DistanceTimeCalculator distancetimecalculator_msg;

  Distance_TimeCalculator Dt = Distance_TimeCalculator(&n);
  distancetimecalculator_pub = n.advertise<freeway_msgs::DistanceTimeCalculator>("freeway/distancetimecalculator", 100);
  std::string root_path=ros::package::getPath("fw_rev_04_navigation");
  std::string config_path=root_path.append("/config/initial_pose_params.yaml");

  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    ros::Time cur_time = ros::Time::now();
    // ros::master::V_TopicInfo master_topics;
    // ros::master::getTopics(master_topics);

    // for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
    //   const ros::master::TopicInfo& info = *it;
    //   std::cout << "topic_" << it - master_topics.begin() << ": " << info.name << std::endl;
    // }
    // if ((ros::Time::now().toSec() - Dt.move_base_GlobalPlanner_plan_Time) < 2.0) { Dt.status_flag=1; }
    // else { Dt.status_flag=0; Dt.remaining_time = 0.0; Dt.msg_global_path_distance=0.0; Dt.first_global_path_distance=0.0; Dt.traveled_distance=0.0; Dt.first_time=0; Dt.remaining_percentage=0.0; Dt.global_path_flag=false;}
    Dt.check_update_time(cur_time);
    Dt.check_block_time(cur_time);
    if(fix_pose) Dt.check_pose_time(cur_time, config_path);

    distancetimecalculator_msg.distance_remaining = Dt.r_msg_global_path_distance();
    distancetimecalculator_msg.arrival_time = Dt.r_remaining_time();
    distancetimecalculator_msg.distance_robot_traveled = Dt.r_traveled_distance();
    distancetimecalculator_msg.remaining_distance_percentage = Dt.r_remaining_percentage();
    distancetimecalculator_msg.status_info = Dt.r_status_info_();
    
    distancetimecalculator_pub.publish(distancetimecalculator_msg);
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

