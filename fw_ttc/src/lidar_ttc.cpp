#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <sstream>

#define signal_distance 0.4
#define signal_distance_teb 0.45
#define signal_release 0.55

#define RP_LIDAR_S2 true


class Freeway_Lidar_Ttc
{
  public:
    Freeway_Lidar_Ttc(ros::NodeHandle *n)
    {
      input_scan_sub = n->subscribe("/scan_rp_filtered", 50, &Freeway_Lidar_Ttc::scan_update, this);

      signal_checker = 0;
      signal2_checker = 0;
      signal_checker_teb = 0;
      signal2_checker_teb = 0;
    }

    bool isFrontObstacleDetected() const
    {
      return (signal_checker > 0 && signal_checker <= 30);
    }

    bool isFrontObstacleTebDetected() const
    {
      return (signal_checker_teb > 0 && signal_checker_teb <= 30);
    }

  private:
    ros::Subscriber input_scan_sub;
    uint32_t signal_checker;
    uint32_t signal2_checker;
    uint32_t signal_checker_teb;
    uint32_t signal2_checker_teb;

        void scan_update(const sensor_msgs::LaserScan& input_scan)
    {

      if(RP_LIDAR_S2) {
      if (!input_scan.ranges.empty()) {
          float res_per_deg = (int)input_scan.ranges.size() / (float)360.0;
          float las_mid_ran = res_per_deg * (float)180.0;
          float deg_15 = floor(res_per_deg * (float)15.0);
          float* ran_arr = new float[8*int(floor(deg_15))]();

          for (unsigned int i = 0; i < 4*int(floor(deg_15)); i++) {
            if(input_scan.ranges[i] < signal_distance) {
              signal_checker++;
            }
    	if(input_scan.ranges[i] < signal_distance_teb) {
             signal_checker_teb++;
            }
            ran_arr[i]=input_scan.ranges[i];
          }

          for (unsigned int i = input_scan.ranges.size()-1 - 4*int(floor(deg_15)); i < input_scan.ranges.size()-1; i++) {
            if(input_scan.ranges[i] < signal_distance) {
              signal_checker++;
            }
    	if(input_scan.ranges[i] < signal_distance_teb) {
    	  signal_checker_teb++;
    	}
            if (signal_checker >= 30) {
              signal_checker = 30;
              //ROS_INFO("Emergency_Safety_LiDAR Detection!!!!!!!!!!!!!!!!!!!!");
            }
    	if (signal_checker_teb >= 30) {
    	  signal_checker_teb = 30;
    	}
            ran_arr[(i+(4*int(floor(deg_15))))-(input_scan.ranges.size()-1 - 4*int(floor(deg_15)))]=input_scan.ranges[i];
          }      
          for (unsigned int i = 0; i < 8*int(floor(deg_15))-1; i++) {
            if (ran_arr[i] >= signal_release) {
              signal2_checker++;
    	      signal2_checker_teb++;
            }
          }
          if (signal2_checker >= 8*int(floor(deg_15))-2) {
            signal2_checker =0;
            signal_checker = 0;
    	    signal2_checker_teb = 0;
    	    signal_checker_teb = 0;
          }
          else if (signal2_checker < 8*int(floor(deg_15))-2) { signal2_checker = 0; signal2_checker_teb = 0; }

          delete [] ran_arr;
         }
    }
      
      else if(!RP_LIDAR_S2) {
        if (!input_scan.ranges.empty()) {
            float res_per_deg = (int)input_scan.ranges.size() / (float)360.0;
            float las_mid_ran = res_per_deg * (float)180.0;
            float deg_15 = floor(res_per_deg * (float)15.0);
            float* ran_arr = new float[8*int(floor(deg_15))]();

            for (unsigned int i = int(floor(las_mid_ran))-4*int(floor(deg_15)); i < int(floor(las_mid_ran))+4*int(floor(deg_15)); i++) {
              if(input_scan.ranges[i] < signal_distance) {
                signal_checker++;
              }
              if(input_scan.ranges[i] < signal_distance_teb) {
                signal_checker_teb++;
                }
                
              if (signal_checker >= 30) {
                signal_checker = 30;
              }

              if (signal_checker_teb >= 30) {
                signal_checker_teb = 30;
              }
              
              ran_arr[i-(int(floor(las_mid_ran))-4*int(floor(deg_15)))]=input_scan.ranges[i];
            }

            for (unsigned int i = 0; i < 8*int(floor(deg_15))-1; i++) {
              if (ran_arr[i] >=signal_release) {
                signal2_checker++;
                signal2_checker_teb++;
              }

            }
            if (signal2_checker >= 8*int(floor(deg_15))-2) {
              signal2_checker =0;
              signal_checker = 0;
    	      signal2_checker_teb = 0;
    	      signal_checker_teb = 0;
            }

            else if (signal2_checker < 8*int(floor(deg_15))-2) { signal2_checker = 0; signal2_checker_teb = 0; }

            delete [] ran_arr;
           }
         }
       }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "freeway_lidar_ttc_node");
  ros::NodeHandle n;
  ros::Publisher front_obstacle_pub = n.advertise<std_msgs::Bool>("/freeway/front_obstacle", 10);
  ros::Publisher front_obstacle_teb_pub = n.advertise<std_msgs::Bool>("/freeway/front_obstacle_teb", 10);
  Freeway_Lidar_Ttc FLT = Freeway_Lidar_Ttc(&n);
  ros::Rate loop_rate(26);

  while (ros::ok())
  {
    std_msgs::Bool front_obstacle_detected;
    std_msgs::Bool front_obstacle_teb_detected;

    front_obstacle_detected.data = FLT.isFrontObstacleDetected();
    front_obstacle_teb_detected.data = FLT.isFrontObstacleTebDetected();

    front_obstacle_pub.publish(front_obstacle_detected);
    front_obstacle_teb_pub.publish(front_obstacle_teb_detected);    

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
