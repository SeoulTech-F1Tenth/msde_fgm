#include "msde_fgm/util_msde.h"

#include "nav_msgs/Odometry.h"

#include <cmath>

namespace util_msde{

    Point_xy quanternion2xyt(const nav_msgs::Odometry::ConstPtr& odom_data)
    {
        Point_xy xyPoint;
        double qx, qy, qz, qw;
        double siny_cosp, cosy_cosp;

        qx = odom_data -> pose.pose.orientation.x;
        qy = odom_data -> pose.pose.orientation.y;
        qz = odom_data -> pose.pose.orientation.z;
        qw = odom_data -> pose.pose.orientation.w;

        siny_cosp = 2.0 * (qw*qz + qx*qy);
        cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz);

        xyPoint.x = odom_data -> pose.pose.position.x;
        xyPoint.y = odom_data -> pose.pose.position.y;
        xyPoint.theta = atan2(siny_cosp, cosy_cosp);

        return xyPoint;
    }


    Point_rt quanternion2rt(const nav_msgs::Odometry::ConstPtr& odom_data)
    {
        Point_rt rtPoint;
        double x, y;

        x = odom_data -> pose.pose.position.x;
        y = odom_data -> pose.pose.position.y;

        rtPoint.r = sqrt(x*x + y*y);
        rtPoint.theta = atan2(y, x);

        return rtPoint;
    }


    Point_rt xyt2rt(Point_xy original_point)
    {
        Point_rt rtPoint;
        double x, y;

        x = original_point.x;
        y = original_point.y;

        rtPoint.r = sqrt(x*x + y*y);
        rtPoint.theta = atan2(y, x) - (PI/2);

        return rtPoint;
        
    }
    
    Point_xy transformPoint(Point_xy robot_pos, Point_xy target)
    {
        Point_xy tf_point;
        double dx, dy, theta, dtheta;
        
        theta = PI/2 - robot_pos.theta;

        dx = target.x - robot_pos.x;
        dy = target.y - robot_pos.y;
        dtheta = target.theta + theta;

        tf_point.x = dx * cos(theta) - dy * sin(theta);
        tf_point.y = dx * sin(theta) + dy * cos(theta);
        tf_point.theta = dtheta;

        return tf_point;
    }

    double getDistance(Point_xy A, Point_xy B)
    {
        double dx, dy;
        dx = A.x - B.x;
        dy = A.y - B.y;
        return sqrt( pow(dx,2) + pow(dy,2));
    }


}
