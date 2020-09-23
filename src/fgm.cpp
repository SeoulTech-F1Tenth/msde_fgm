#include "msde_fgm/fgm.h"

#include "msde_fgm/util_msde.h"

#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

#include <iostream>
#include <vector>
#include <string.h>

namespace fgm{


    float scan_angle_min=0;
    float scan_angle_max=0;
    float scan_angle_increment=0;
    int scan_range_size=0;
    int range_mid_idx=0;

    // parameter for driving
    float robot_scale=0;
    float threshold=0;
    int gap_size=0;
    float filter_scale=0;


    FGM::FGM(const ros::NodeHandle h)
    :nh_c(h), loop_rate(100), current_rp_idx(0)
    {
        ROS_INFO("start fgm node");

        // get reference point
        nh_c.getParam("/msde_driving/reference_point/filepath_1", path_temp_1);
        path_pack = ros::package::getPath("msde_fgm");
        path_1 = path_pack + path_temp_1;
        nh_c.getParam("/msde_driving/reference_point/distance", rf_distance);
        filepath_1 = new char[ path_1.length()+1 ];
        strcpy(filepath_1, path_1.c_str());
        get_reference_point();

        // get parameters for name of topics
        nh_c.getParam("/msde_driving/topic/scan", scan_topic_name);
        nh_c.getParam("/msde_driving/topic/odom", odom_topic_name);
        nh_c.getParam("/msde_driving/topic/nav", drive_topic_name);

        // get parameters for driving configurations
        nh_c.getParam("/msde_driving/driving/robot_scale", robot_scale);
        nh_c.getParam("/msde_driving/driving/threshold", threshold);
        nh_c.getParam("/msde_driving/driving/gap_size", gap_size);
        nh_c.getParam("/msde_driving/driving/filter_scale", filter_scale);
        nh_c.getParam("/msde_driving/driving/speed_max", speed_max);
        nh_c.getParam("/msde_driving/driving/speed_min", speed_min);
        speed_coef = (4*(speed_max-speed_min))/(pow(PI,2));

        // get parameter for theta selection gain values
        nh_c.getParam("/msde_driving/driving/gap_theta_gain", gap_theta_gain);
        nh_c.getParam("/msde_driving/driving/ref_theta_gain", ref_theta_gain);
        


        sub_init_scan = nh_c.subscribe(scan_topic_name, 1, &FGM::sub_initLidarData, this);
        // subscriber and publisher init
        sub_scan = nh_c.subscribe(scan_topic_name, 1, &FGM::subCallback_scan, this);
        sub_odom = nh_c.subscribe(odom_topic_name, 1, &FGM::subCallback_odom, this);
        pub_ack = nh_c.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic_name, 1);

        // test publisher
        pub_scan_filtered = nh_c.advertise<sensor_msgs::LaserScan>("laserscan_gap", 1);
        pub_dp_mark = nh_c.advertise<visualization_msgs::Marker>("/desired_point/marker", 1);


        // Ackermann Driving message init
        pub_ack_msg.header.stamp = ros::Time::now();
        pub_ack_msg.header.frame_id = "ego_racecar/base_link";
        pub_ack_msg.drive.speed = 0;
        pub_ack_msg.drive.acceleration = 0;
        pub_ack_msg.drive.jerk = 0;
        pub_ack_msg.drive.steering_angle = 0;
        pub_ack_msg.drive.steering_angle_velocity = 0;

    }

    FGM::~FGM()
    {
        delete []scan_origin;
        delete []scan_filtered;
        delete []filepath_1;
        delete []rf_points;
    }

    void FGM::get_reference_point()
    {
        ROS_INFO("get reference point");
        rf_num = 0;

        fs.open(filepath_1, std::ios::in);
        ROS_INFO("file opened");
        while(!fs.eof())
        {
            getline(fs, str_buf, ',');
            getline(fs, str_buf);
            rf_num++;
        }
        fs.close();
        ROS_INFO("%d points", rf_num);

        rf_points = new util_msde::Point_xy[rf_num];

        fs.open(filepath_1, std::ios::in);
        ROS_INFO("open file");
        util_msde::Point_xy point_temp;

        int i=0;
        while(!fs.eof())
        {
            getline(fs, str_buf, ',');
            point_temp.x = std::strtof(str_buf.c_str(), 0);
            getline(fs, str_buf);
            point_temp.y = std::strtof(str_buf.c_str(), 0);
            point_temp.theta = 0;
            rf_points[i] = point_temp;
            i++;
        }
        fs.close();
        ROS_INFO("close file");
    } // end get reference point

    void FGM::find_desired_rp()
    {
        double temp_distance;
        int current_rp_idx_temp = current_rp_idx;

        nearest_distance = util_msde::getDistance(rf_points[current_rp_idx_temp], current_position);
                        
        while(1)
        {
            current_rp_idx_temp++;

            if(current_rp_idx_temp >= rf_num) current_rp_idx_temp = 0;

            temp_distance = util_msde::getDistance(rf_points[current_rp_idx_temp], current_position);

            if(temp_distance < nearest_distance)
            {
                nearest_distance = temp_distance;
                current_rp_idx = current_rp_idx_temp;
            }
            else if(temp_distance > (nearest_distance + rf_distance*1.2) || (current_rp_idx_temp == current_rp_idx)){ break; } 
        }

        temp_distance = 0;
        int idx_temp = current_rp_idx;
        while(1)
        {
            if(idx_temp >= rf_num) idx_temp = 0;
            temp_distance = util_msde::getDistance(rf_points[idx_temp], current_position);
            if(temp_distance > rf_distance) break;
            idx_temp++;
        }
        reference_point = rf_points[idx_temp];
        util_msde::Point_xy rfpoint_tf;
        rfpoint_tf = transformPoint(current_position, reference_point);
        ref_point_rt = util_msde::xyt2rt(rfpoint_tf);

        /*
        Dp_Marker.header.frame_id = "map";
        Dp_Marker.header.stamp = ros::Time();
        Dp_Marker.ns = "msde_fgm";
        Dp_Marker.id = 1;
        Dp_Marker.type = visualization_msgs::Marker::SPHERE;
        Dp_Marker.action = visualization_msgs::Marker::ADD;
        Dp_Marker.pose.position.x = reference_point.x;
        Dp_Marker.pose.position.y = reference_point.y;
        Dp_Marker.pose.position.z = 0.1;
        Dp_Marker.pose.orientation.x = 0.0;
        Dp_Marker.pose.orientation.y = 0.0;
        Dp_Marker.pose.orientation.z = 0.0;
        Dp_Marker.pose.orientation.w = 1.0;
        Dp_Marker.scale.x = 0.1;
        Dp_Marker.scale.y = 0.1;
        Dp_Marker.scale.z = 0.1;
        Dp_Marker.color.a = 1.0; // Don't forget to set the alpha!
        Dp_Marker.color.r = 1.0;
        Dp_Marker.color.g = 0.0;
        Dp_Marker.color.b = 0.0;
        //only if using a MESH_RESOURCE marker type:
        pub_dp_mark.publish( Dp_Marker );
        */

    } // end finding reference point
    

    void FGM::start_driving()
    {
        GAP gap_obj;

        ROS_INFO("starting");
        ros::Duration(5.0).sleep();
//        ros::Duration five_seconds = ros::Duration(5.0);
        ROS_INFO("start driving");

        while(ros::ok())
        {
            ros::spinOnce();
//            ros::topic::waitForMessage<sensor_msgs::LaserScan>(scan_topic_name);
            if(scan_range_size == 0) continue;
            // init ackermann message header
//            pub_ack_msg.header.stamp = ros::Time::now();
//            pub_ack_msg.header.frame_id = "base_link";

            //find gap
            gap_obj.gap_finding(scan_filtered);
            gap_obj.for_gap_finding(scan_filtered);
//            gap_obj.print_gapinfo();
//            ROS_INFO("current position: (%f, %f)", current_position.x, current_position.y);

//            ROS_INFO("reference point: (%f, %f)", ref_point_rt.r, ref_point_rt.theta);

            //test
            Gap goal_gap = gap_obj.get_nearest_gap(ref_point_rt);
//            Gap goal_gap = gap_obj.get_maximum_gap();
//            drive_test(goal_gap);
            drive_with_ref(goal_gap);

            std::cout<<std::endl;
            //std::system("clear");
            loop_rate.sleep();
        }
    }

    void FGM::drive_with_ref(Gap goal)
    {
        // steering angle calculation
        float steering_angle;
        float max_angle = (goal.max_idx - range_mid_idx)*scan_angle_increment;
        float ref_angle = ref_point_rt.theta;
        
        // get range data in ref_direction
        int step = (int)(ref_angle/scan_angle_increment);
        int ref_idx = range_mid_idx + step;
        /*
        // 10degree range selection : 18
        int range = (int)((PI/15)/scan_angle_increment);
        int st_idx, en_idx;
        if( (ref_idx + range/2) < 0 )
        {
            st_idx = 0;
            en_idx = 5;
        }
        else if( (ref_idx - range/2) >= scan_range_size )
        {
            st_idx = scan_range_size - 6;
            en_idx = scan_range_size - 1;
        }
        else if( (ref_idx - range/2) < 0 )
        {
            st_idx = 0;
            en_idx = (int)(ref_idx + range/2);
        }
        else if( (ref_idx + range/2) >= scan_range_size )
        {
            st_idx = (int)(ref_idx - range/2);
            en_idx = scan_range_size - 1;
        }
        else
        {
            st_idx = (int)(ref_idx - range/2);
            en_idx = (int)(ref_idx + range/2);
        }

        float range_sum=0;
        float dmin=0;
        int i;
        for(i=st_idx; i<=en_idx; i++)
        {
            range_sum += scan_filtered[i];
        }
        dmin = range_sum / (en_idx-st_idx+1);
        */

        // float range_min_values[10];
        float temp_avg=0;
        float dmin = 0;
        int i, j;
        for(i=0; i<5; i++)
        {
            dmin += scan_filtered[i];
        }
        dmin /= 5;
        for(i=1; i<scan_range_size-5; i++)
        {
            for(j=0; j<5; j++)
            {
                temp_avg += scan_filtered[i+j];
            }
            temp_avg /= 5;
            if(dmin > temp_avg) {
                dmin = temp_avg;
            }
            temp_avg = 0;
        }





        float controlled_angle = ( (gap_theta_gain/dmin)*max_angle + ref_theta_gain*ref_angle )/(gap_theta_gain/dmin + ref_theta_gain);
        float distance = 1.0;
        float path_radius = distance / (2 * sin(controlled_angle));
        steering_angle = (float)atan(RACECAR_LENGTH/path_radius);
        


        // speed control
        double speed;
        double g = -3.5;
        float abs_angle = fabs(steering_angle);
        if( abs_angle > (PI/4)){
            speed = speed_min;
        } else {
            speed = (speed_max-speed_min)*exp(g*abs_angle)+speed_min;
//            speed = -1*(4/PI)*(speed_max - speed_min)*abs_angle + speed_max;
//            speed = speed_max;
        }


        // publish ackermann message
        pub_ack_msg.header.stamp = ros::Time::now();
        pub_ack_msg.header.frame_id = "ego_racecar/base_link";
        pub_ack_msg.drive.steering_angle = steering_angle;
        pub_ack_msg.drive.steering_angle_velocity = 8;
        pub_ack_msg.drive.speed = speed;
        pub_ack_msg.drive.acceleration = 8;
        pub_ack_msg.drive.jerk = 8;

        pub_ack.publish(pub_ack_msg);

        /*
        ROS_INFO("dmin : %f", dmin);
        ROS_INFO("ref_angle : %f, gap_angle : %f", ref_angle, max_angle);
        ROS_INFO("speed: %f,  angle: %f", speed, controlled_angle);
        */

        /*
        scan_filter_test.ranges.clear();
        for(i=0; i<scan_range_size; i++) {
            if( i > goal.start_idx && i < goal.end_idx)
            {
                scan_filter_test.ranges.push_back(scan_filtered[i]);
            } else {
                scan_filter_test.ranges.push_back(0);
            }
        }
        pub_scan_filtered.publish(scan_filter_test);
        */

    } // finish drive function




    // drive test without reference point
    void FGM::drive_test(Gap goal)
    {
        float steering_angle;
        float max_angle = (goal.max_idx - range_mid_idx)*scan_angle_increment;
        // float mid_angle = ((goal.start_idx + goal.end_idx)/2 - range_mid_idx)*scan_angle_increment;

        float distance = 1.0;
        float path_radius = distance / (2 * sin(max_angle));
        steering_angle = atan(RACECAR_LENGTH/path_radius);
        
        float speed;
        if( fabs(max_angle) > (PI/2)){
            speed = speed_min;
        } else {
            speed = (float)(-(2/PI)*(speed_max-speed_min)*fabs(max_angle) + speed_max);
            //speed = speed_max;
        }

        pub_ack_msg.drive.steering_angle = steering_angle;
        pub_ack_msg.drive.steering_angle_velocity = 0;
        pub_ack_msg.drive.speed = speed;
        pub_ack_msg.drive.acceleration = 8;
        pub_ack_msg.drive.jerk = 0;

        ROS_INFO("speed: %f,  angle: %f", speed, steering_angle);
        pub_ack.publish(pub_ack_msg);

        int i;
        scan_filter_test.ranges.clear();
        for(i=0; i<scan_range_size; i++) {
            if( i > goal.start_idx && i < goal.end_idx)
            {
                scan_filter_test.ranges.push_back(scan_filtered[i]);
            } else {
                scan_filter_test.ranges.push_back(0);
            }
        }
        pub_scan_filtered.publish(scan_filter_test);
    }


    /* get the laser scan data description */
    void FGM::sub_initLidarData(const sensor_msgs::LaserScan::ConstPtr& msg_sub)
    {
        ROS_INFO("GET THE LIDAR DESCRIPTION");
        scan_angle_min = msg_sub -> angle_min;
        scan_angle_max = msg_sub -> angle_max;
        scan_angle_increment = msg_sub -> angle_increment;
        scan_range_size = msg_sub -> ranges.size();
        range_mid_idx = (int)(scan_range_size/2);

        // allocate the scan data array
        scan_origin = new float[ scan_range_size ];
        scan_filtered = new float[ scan_range_size ];

        sub_init_scan.shutdown();
    }

    void FGM::subCallback_scan(const sensor_msgs::LaserScan::ConstPtr& msg_sub)
    {

        if(scan_range_size == 0)
        {
            ROS_INFO("GET THE LIDAR DESCRIPTION");
            scan_angle_min = msg_sub -> angle_min;
            scan_angle_max = msg_sub -> angle_max;
            scan_angle_increment = msg_sub -> angle_increment;
            scan_range_size = msg_sub -> ranges.size();
            range_mid_idx = (int)(scan_range_size/2);

            // allocate the scan data array
            scan_origin = new float[ scan_range_size ];
            scan_filtered = new float[ scan_range_size ];
        }


        int i;
        for(i=0; i<scan_range_size; i++){
            scan_origin[i] = msg_sub -> ranges[i];
            scan_filtered[i] = msg_sub -> ranges[i];
        }
        
        // zero filtering
        for(i=0; i<scan_range_size; i++)
        {
            if(scan_origin[i] == 0)
            {
                float sum = 0;
                int count = 0;
                int j;
                for(j=1; j<=20; j++)
                {
                    if(i-j >= 0){
                        if(scan_origin[i-j] != 0){
                            count++;
                            sum += scan_origin[i-j];
                        }
                    }
                    if(i+j < scan_range_size){
                        if(scan_origin[i+j] != 0){
                            count++;
                            sum += scan_origin[i+j];
                        }
                    }
                }
                scan_origin[i] = sum/count;
                scan_filtered[i] = sum/count;
            }
        }// end zero filtering

        // filtering the scan data
        for(i=0; i<scan_range_size-1; i++)
        {
            // right to left
            if(scan_origin[i]*filter_scale < scan_filtered[i+1])
            {
                float unit_length = scan_origin[i] * scan_angle_increment;
                float filter_num = robot_scale / unit_length;
                int j;
                for(j=1; j<filter_num+1; j++)
                {
                    if(i+j < scan_range_size)
                    {
                        if(scan_filtered[i+j] > scan_origin[i]){
                            scan_filtered[i+j] = scan_origin[i];
                        } else {
                            break;
                        }
                    } else { break; }
                }
            }
            // left to right
            else if(scan_filtered[i] > scan_origin[i+1]*filter_scale)
            {
                float unit_length = scan_origin[i+1] * scan_angle_increment;
                float filter_num = robot_scale / unit_length;
                int j;
                for(j=0; j<filter_num+1; j++)
                {
                    if(i-j > 0)
                    {
                        if(scan_filtered[i-j] > scan_origin[i+1]){
                            scan_filtered[i-j] = scan_origin[i+1];
                        } else {
                            break;
                        }
                    } else { break; }
                }
            }

        } // scan filter end

        /*
        float start_ = range_mid_idx - (PI/2)/scan_angle_increment;
        float end_ = range_mid_idx + (PI/2)/scan_angle_increment;
        for(i=0; i<start_; i++)
        {
            scan_filtered[i] = 0;
        }
        for(i=end_; i<scan_range_size; i++)
        {
            scan_filtered[i] = 0;
        }
        */

        // publish filtered scan data
        scan_filter_test.header = msg_sub -> header;
        scan_filter_test.angle_min = scan_angle_min;
        scan_filter_test.angle_max = scan_angle_max;
        scan_filter_test.angle_increment = scan_angle_increment;
        scan_filter_test.time_increment = msg_sub -> time_increment;
        scan_filter_test.scan_time = msg_sub -> scan_time;
        scan_filter_test.range_min = msg_sub -> range_min;
        scan_filter_test.range_max = msg_sub -> range_max;
        /*
        int i;
        scan_filter_test.ranges.clear();
        for(i=0; i<scan_range_size; i++) {
            scan_filter_test.ranges.push_back(scan_filtered[i]);
        }
        pub_scan_filtered.publish(scan_filter_test);
        */

    }


    void FGM::subCallback_odom(const nav_msgs::Odometry::ConstPtr& msg_sub)
    {
        this->current_position = util_msde::quanternion2xyt(msg_sub);
        find_desired_rp();
    }


    // GAP class
    GAP::GAP()
    {
        this->gaps.reserve(20);

        float theta_for = PI/2.5;
        int for_point = (int)(theta_for/scan_angle_increment);
        this->for_gap.start_idx = (range_mid_idx) - for_point;
        this->for_gap.end_idx = (range_mid_idx) + for_point;
    }

    GAP::~GAP()
    {
        std::vector<Gap>().swap(gaps);
    }

    void GAP::gap_finding(float* scan)
    {
        // clear previous gaps vector
        this->gaps.clear();
        int i, j;
        for(i=0; i<scan_range_size - gap_size; i++)
        {
            if(scan[i] > threshold)
            {
                float start_idx_temp = i;
                float end_idx_temp =i;
                float max_temp = scan[i];
                float max_idx_temp = i;
                // check is gap or not
                for(j=1; j<gap_size; j++)
                {
                    if(scan[++i] > threshold){
                        if(scan[i] > max_temp){
                            max_temp = scan[i];
                            max_idx_temp = i;
                        }
                    } else {
                        max_temp = -1;
                        break;
                    } 
                }//end checking is gap or not

                // is not gap
                if(max_temp == -1){
                    break;
                }
                // is gap 
                else{
                    while( (scan[i] > threshold) && (++i < scan_range_size) )
                    {
                        if(scan[i] > max_temp){
                            max_temp = scan[i];
                            max_idx_temp = i;
                        }
                    }
                    end_idx_temp = i;
                    // push the gap data
                    Gap gap_temp;
                    gap_temp.start_idx = start_idx_temp;
                    gap_temp.end_idx = end_idx_temp;
                    gap_temp.max_idx = max_idx_temp;
                    this->gaps.push_back(gap_temp);
                }

            }
        }// end gap finding loop
    }// end gap finding method

    void GAP::for_gap_finding(float* scan)
    {
        float theta_for = PI/2.5;
        int for_point = (int)(theta_for/scan_angle_increment);
        int start_idx_temp = (range_mid_idx) - for_point;
        int end_idx_temp = (range_mid_idx) + for_point;

        /*
        ROS_INFO("%d, %d", start_idx_temp, end_idx_temp);
        ROS_INFO("%d/ %d", range_mid_idx, scan_angle_increment);
        */

        int i;
        int max_idx_temp=start_idx_temp;
        float max_temp;
        max_temp = scan[start_idx_temp];

        for(i=start_idx_temp; i<end_idx_temp; i++)
        {
            if(max_temp < scan[i]){
                max_temp = scan[i];
                max_idx_temp = i;
            }
        }

        this->for_gap.start_idx = start_idx_temp; 
        this->for_gap.end_idx = end_idx_temp;
        this->for_gap.max_idx = max_idx_temp;
    }


    int GAP::get_gapsize()
    {
        return this->gaps.size();
    }

    Gap GAP::get_for_gap()
    {
        return this->for_gap;
    }

    // get maximum gap
    // return for_gap if there is no gap in gaps
    Gap GAP::get_maximum_gap()
    {
        int num = this->gaps.size();
        if( num == 0 ){
            return this->for_gap;
        }
        else{
            int i;
            int max_gap_idx = 0;
            int max_gap_size = this->gaps[0].end_idx - this->gaps[0].start_idx;
            int temp_gap_size = 0;
            for(i=0; i<num; i++)
            {
                temp_gap_size = this->gaps[i].end_idx - this->gaps[i].start_idx;
                if(temp_gap_size > max_gap_size){
                    max_gap_idx = i;
                    max_gap_size = temp_gap_size;
                }
            }
            return this->gaps[max_gap_idx];
        }

    }// end get maximum gpa method


    // get nearest gap from reference point
    Gap GAP::get_nearest_gap(util_msde::Point_rt ref)
    {
        int num = this->gaps.size();
        if( num == 0 ){
            return this->for_gap;
        }
        else{
            int i;
            int ref_idx;
            int step = (int)(ref.theta/scan_angle_increment);
            ref_idx = range_mid_idx + step;

            int gap_idx = 0;
            int distance;
            
            if(gaps[0].start_idx > ref_idx){
                distance = gaps[0].start_idx - ref_idx;
            } else if(gaps[0].end_idx < ref_idx) {
                distance = ref_idx - gaps[0].end_idx;
            } else {
                distance = 0;
                gap_idx = 0;
            }

            i = 1;
            int temp_distance;
            while(i < num)
            {
                if(gaps[i].start_idx > ref_idx){
                    temp_distance = gaps[i].start_idx - ref_idx;
                    if(temp_distance < distance)
                    {
                        distance = temp_distance;
                        gap_idx = i;
                    }
                } else if(gaps[i].end_idx < ref_idx) {
                    temp_distance = ref_idx - gaps[i].end_idx;
                    if(temp_distance < distance)
                    {
                        distance = temp_distance;
                        gap_idx = i;
                    }
                } else {
                    temp_distance = 0;
                    distance = 0;
                    gap_idx = i;
                    break;
                }
                i++;
            }
//            this->print_gapinfo();
            return this->gaps[gap_idx];
        }
    }// end get nearest gap




    void GAP::print_gapinfo()
    {
        ROS_INFO("for gap s: %d, e: %d, max: %d", this->for_gap.start_idx, this->for_gap.end_idx, this->for_gap.max_idx);
        int num = this->gaps.size();
        ROS_INFO("number of gap : %d", num);
        int i;
        for(i=0; i<num; i++){
            ROS_INFO("start: %d,  end: %d, maximum: %d", this->gaps[i].start_idx, this->gaps[i].end_idx, this->gaps[i].max_idx);
        }
    }


}

