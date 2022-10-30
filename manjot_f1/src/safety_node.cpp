#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <algorithm>
// TODO: include ROS msg type headers and libraries

class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    double speed, scan_beams, ttc_thre;
    ros::Publisher b;
    ros::Publisher bb;
    ros::Subscriber s_msg;
    ros::Subscriber o_msg;
    // TODO: create ROS subscribers and publishers

public:
    Safety() {
        n = ros::NodeHandle();
        speed = 0.0;

        // Getting no. of scan beams of the lidar from the parameter server
        if(!n.getParam("safety_node/scan_beams", scan_beams))
        {
            ROS_ERROR("scan_beams not got");
        }

        // Getting the TTC threshould from the parameter server
        if(!n.getParam("safety_node/ttc_thre", ttc_thre))
        {
            ROS_ERROR("ttc_thre not got");
        }
        
        /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.
        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages
        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        */

        // TODO: create ROS subscribers and publishers
        b = n.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 1);
        bb = n.advertise<std_msgs::Bool>("/brake_bool", 1);
        s_msg = n.subscribe("/scan", 1, &Safety::scan_callback, this);
        o_msg = n.subscribe("/odom", 1, &Safety::odom_callback, this);
        
    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        // TODO: update current speed
        speed = odom_msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        // TODO: calculate TTC
        double theta, ttc;
        ackermann_msgs::AckermannDriveStamped drive_msg;
        std_msgs::Bool brake_msg;

        // iterating through the range of laser scans to calculate the angle of the scan
        for(int i = 0; i < scan_beams; i++)
        {   
            //calculating theta by multiplying the angle increment of the lidar
            theta = scan_msg->angle_min + i*scan_msg->angle_increment;

            //calculting Time To Collision(TTC) ttc = range/speedinthesirectionofrange
            ttc = scan_msg->ranges[i] / std::abs(speed*cos(theta));

            //checking whether calculated ttc in less than threshould ttc
            if(!std::isinf(ttc) && ttc < ttc_thre)
            {
                // if its less than sending 0 velocity to the brake topic
                // which is subscribed by the mux node.
                drive_msg.header.seq = i;
                drive_msg.drive.speed = 0.0;
                drive_msg.drive.acceleration = 0.0;
                drive_msg.drive.jerk = 0.0;
                b.publish(drive_msg);

                //also sending true in the brake bool topic 
                // which is subscribed by the behavior_controller node
                brake_msg.data = true;
                bb.publish(brake_msg);
            }
        }
        // TODO: publish drive/brake message
    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    ros::spin();
    return 0;
}