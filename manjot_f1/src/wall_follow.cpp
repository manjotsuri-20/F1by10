#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
// #define ANGLE_RANGE 270
#define CAR_LENGTH 0.50 //0.5 metres
#define DESIRED_DISTANCE_FROM_LEFT 0.7
#define kp 0.06
#define kd 0.00031
#define ki 1.0

class WallFollow
{
    private:
    ros::NodeHandle n;
    ros::Subscriber scan_sub;
    ros::Publisher drive_pub;
    double inte_err, diff_err, pre_err;

    public:
    WallFollow()
    {
        n = ros::NodeHandle();
        scan_sub = n.subscribe("/scan", 1, &WallFollow::lidar_callback, this);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1);
        inte_err = 0.0;
        diff_err = 0.0;
        pre_err = 0.0;
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
        // this->diff_err = error - this->pre_err;
        // this->inte_err = this->inte_err + error;
        // this->pre_err = error;
        // ROS_INFO("err = %f, diff_err = %f, int_err = %f", error, this->diff_err, this->inte_err);
        // angle = (-kp * error) + (-ki * this->inte_err) + (-kd * diff_err);
        // ROS_INFO("Angle %f", angle);

        angle = ((kd*error*error)+(kp*error)+ki)/((error*error*error)+((10+kd)*error*error)+((20+kp)*error)+ki);

        if(angle > -0.174533 && angle < 0.174533) velocity = 1.5;
        else if (angle >= 0.174533 && angle <= -0.174533 && angle < 0.349 && angle > -0.349) velocity = 1.0;
        else velocity = 0.5;

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
        d = b * cos(alpha) + CAR_LENGTH * sin(alpha);
        return d - DESIRED_DISTANCE_FROM_LEFT;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wall_follow");
    WallFollow wf;
    ros::spin();
    return 0;
}