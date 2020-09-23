#ifndef __HEADER_fgm_
#define __HEADER_fgm_

#include "ros/ros.h"
#include "ros/package.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "visualization_msgs/Marker.h"

#include "msde_fgm/util_msde.h"

#include <string.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <time.h>

#define PI 3.141592
#define RACECAR_LENGTH 0.325

namespace fgm
{

    // parameter of laser scan data
    extern float scan_angle_min;
    extern float scan_angle_max;
    extern float scan_angle_increment;
    extern int scan_range_size;
    extern int range_mid_idx;

    // parameter for driving
    extern float robot_scale;
    extern float threshold;
    extern int gap_size;
    extern float filter_scale;


    typedef struct _Gap{
        int start_idx;
        int end_idx;
        int max_idx;
    } Gap;


    class FGM{
    private:
        ros::NodeHandle nh_c;
        ros::Rate loop_rate;
        ros::Subscriber sub_odom;
        ros::Subscriber sub_scan;
        ros::Publisher pub_ack;
        ros::Subscriber sub_init_scan;

        // parameter for topics
        std::string scan_topic_name;
        std::string odom_topic_name;
        std::string drive_topic_name;

        // driving
        float speed_max;
        float speed_min;
        float speed_coef;
        float gap_theta_gain;
        float ref_theta_gain;


        // Lidar data
        float * scan_origin;
        float * scan_filtered;

        // messages
        ackermann_msgs::AckermannDriveStamped pub_ack_msg;
        sensor_msgs::LaserScan scan_data;
        // message temp
        //sensor_msgs::MultiEchoLaserScan scan_filter_test;
        sensor_msgs::LaserScan scan_filter_test;
        ros::Publisher pub_scan_filtered;


        // reference point
        std::fstream fs;
        std::string str_buf;
        std::string path_pack;
        std::string path_temp_1;
        std::string path_1;
        char* filepath_1;
        util_msde::Point_xy* rf_points;
        util_msde::Point_xy reference_point;
        util_msde::Point_rt ref_point_rt;
        int rf_num;
        double rf_distance;
        double nearest_distance;
        // position of robot
        util_msde::Point_xy current_position;
        int current_rp_idx;


        visualization_msgs::Marker Dp_Marker;
        ros::Publisher pub_dp_mark;


    public:
        FGM(const ros::NodeHandle h);
        ~FGM();
        void start_driving();
        void drive_test(Gap goal);
        void drive_with_ref(Gap goal);
        // reference points control
        void get_reference_point();
        void find_desired_rp();
        // sub callback
        void subCallback_scan(const sensor_msgs::LaserScan::ConstPtr& msg_sub);
        void subCallback_odom(const nav_msgs::Odometry::ConstPtr& msg_sub);
        void sub_initLidarData(const sensor_msgs::LaserScan::ConstPtr& msg_sub);

    };

    class GAP{
    private:
        std::vector<Gap> gaps;
        Gap for_gap;

    public:
        GAP();
        ~GAP();
        void gap_finding(float* scan);
        void print_gapinfo();
        void for_gap_finding(float* scan);
        int get_gapsize();
        Gap get_for_gap();
        Gap get_maximum_gap();
        Gap get_nearest_gap(util_msde::Point_rt ref);
    };

}



#endif
