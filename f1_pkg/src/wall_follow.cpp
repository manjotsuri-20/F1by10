#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

class WallFollow
{
    private:
    ros::NodeHandle n;
    ros::Subscriber scan_sub;
    ros::Publisher drive_pub;
    double car_length, desired_distance_from_left, kp, ki, kd;

    public:
    WallFollow()
    {
        n = ros::NodeHandle();
        scan_sub = n.subscribe("/scan", 1, &WallFollow::lidar_callback, this);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1);
        if(!n.getParam("wall_follow/car_length", car_length))
        {
            ROS_ERROR("car_length not found");
        }
        if(!n.getParam("wall_follow/desired_distance_from_left", desired_distance_from_left))
        {
            ROS_ERROR("desired distance not specified");
        }
        if(!n.getParam("wall_follow/kp", kp))
        {
            ROS_ERROR("value of kp not specified not specified");
        }
        if(!n.getParam("wall_follow/kd", kd))
        {
            ROS_ERROR("value of kd not specified not specified");
        }
        if(!n.getParam("wall_follow/ki", ki))
        {
            ROS_ERROR("value of ki not specified not specified");
        }
    }

    void lidar_callback(const sensor_msgs::LaserScan &scan_msg)
    {
        //replace the error returned by follow_left
        //send error to pid_control
        double theta_a, theta_b, a, b, theta, alpha;
        double error; 
        int i;

        i = 810;
        theta_b = scan_msg.angle_min + i*scan_msg.angle_increment;
        if(std::isnan(scan_msg.ranges[i])) b = scan_msg.range_min;
        else if(std::isinf(scan_msg.ranges[i])) b = scan_msg.range_max;
        else b = scan_msg.ranges[i];
        i = 700;
        theta_a = scan_msg.angle_min + i*scan_msg.angle_increment;
        if(std::isnan(scan_msg.ranges[i])) a = scan_msg.range_min;
        else if(std::isinf(scan_msg.ranges[i])) a = scan_msg.range_max;
        else a = scan_msg.ranges[i]; 

        theta = std::abs(theta_b - theta_a);
        error = this->follow_left(theta, a, b);

        this->pid_control(error);
        
    }

    void pid_control(double error)
    {
        double angle, velocity;
        ackermann_msgs::AckermannDriveStamped drive_msgs;

        //use kp, ki and kd to implement a PID controller for
        angle = ((kd*error*error)+(kp*error)+ki)/((error*error*error)+((10+kd)*error*error)+((20+kp)*error)+ki);

        //calculating the velocity from the turning angles
        if(angle > -0.174533 && angle < 0.174533) velocity = 1.5;
        else if (angle >= 0.174533 && angle <= -0.174533 && angle < 0.349 && angle > -0.349) velocity = 1.0;
        else velocity = 0.5;

        //publishing the data to the /nav topic 
        drive_msgs.header.stamp = ros::Time::now();
        drive_msgs.header.frame_id = "laser";
        drive_msgs.drive.steering_angle = angle;
        drive_msgs.drive.speed = velocity;
        drive_pub.publish(drive_msgs);
    }

    double follow_left(double theta, double a, double b)
    {
        //follow left wall as per the algorithm
        double alpha, d;
        alpha = atan2((a*cos(theta) - b), (a*sin(theta)));
        d = b * cos(alpha) + car_length * sin(alpha);
        return d - desired_distance_from_left;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wall_follow");
    WallFollow wf;
    ros::spin();
    return 0;
}