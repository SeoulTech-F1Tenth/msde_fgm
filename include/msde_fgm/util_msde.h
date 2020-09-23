#ifndef __HEADER_util_msde_
#define __HEADER_util_msde_

#include "nav_msgs/Odometry.h"
#include <cmath>

#define PI 3.141592


namespace util_msde{

    typedef struct _Point_xy{
        double x;
        double y;
        // direction of the point
        double theta;
    } Point_xy;

    typedef struct _Point_rt{
        double r;
        double theta;
    } Point_rt;


    // chang point expresion method
    Point_xy quanternion2xyt(const nav_msgs::Odometry::ConstPtr& odom_data);
    Point_rt quanternion2rt(const nav_msgs::Odometry::ConstPtr& odom_data);
    Point_rt xyt2rt(Point_xy original_point);
//    Point_xy rt2xyt(double r, double t);
    Point_xy transformPoint(Point_xy robot_pos, Point_xy target);
    double getDistance(Point_xy A, Point_xy B);


}



#endif
