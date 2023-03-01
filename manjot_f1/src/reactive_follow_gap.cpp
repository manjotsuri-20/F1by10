#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#define car_width 0.2032 //metres
#define obstacle_thresh 1.0

/*
    Lidar Readings array indices
                540
            810     270
                 0
*/

class reactiveFollowGap
{
    private:
        ros::NodeHandle n_rea;
        ros::Publisher drive_pub;
        ros::Subscriber lidar_sub;

        std::string lidarscan_topic;
        std::string drive_topic;

        double kp, ki, kd;

    public:
        reactiveFollowGap()
        {
            n_rea = ros::NodeHandle();

            //Topics and Subscribers, Publishers
            lidarscan_topic = "/scan";
            drive_topic = "/nav";

            drive_pub = n_rea.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
            lidar_sub = n_rea.subscribe(lidarscan_topic, 1, &reactiveFollowGap::lidar_callback, this);


            if(!n_rea.getParam("wall_follow/kp", kp))
            {
                ROS_ERROR("value of kp not specified not specified");
            }
            if(!n_rea.getParam("wall_follow/kd", kd))
            {
                ROS_ERROR("value of kd not specified not specified");
            }
            if(!n_rea.getParam("wall_follow/ki", ki))
            {
                ROS_ERROR("value of ki not specified not specified");
            }
        }

        std::vector<float> preprocess_lidar(std::vector<float> range)
        {
            //Preprocess the LiDAR scan array. Expert implementation includes:
            //1.Setting each value to the mean over some window
            //2.Rejecting high values (eg. > 3m)

            std::vector<float> for_range;
            int start = 432, end = 648;
            for(int i = start; i < end ; i ++ )
            {
                for_range.emplace_back(range[i]);
            }
            return for_range;
        }

        void find_max_gap(std::vector<float> range)
        {
            //Return the start and end index of the max gap in free_space_ranges
            

        }

        double find_best_point(std::vector<float> range, float inc)
        {
            //Start_i & end_i are start and end indicies of max-gap range, respectively
            //Return index of best point in ranges
	        //Naive: Choose the furthest point within ranges and go there

            auto dis = std::distance(range.begin(), std::max_element(range.begin(), range.end()));
            int diff = dis - range.size()/2;
            double angle = diff * inc;
            std::cout << dis << " " << range.size()/2 << " " << diff << " " << angle << std::endl;
            return angle;
        }

        void lidar_callback(const sensor_msgs::LaserScan &scan_msg)
        {
            //Process each LIDAR scan as per the Follow Gap algorithms 
            //& publish an AckermannDriveStamped Message 

            double angle = 0.0;
            auto range = scan_msg.ranges;

            //Preprocessed ranges
            auto proc_ranges = this->preprocess_lidar(range);

            //Find Closest point to lidar
            auto dis = std::distance(proc_ranges.begin(), std::min_element(proc_ranges.begin(), proc_ranges.end()));
            if(proc_ranges[dis] < obstacle_thresh)
            {
                //Eliminate all points inside 'bubble' (set them to zero)
                int i = 0;
                int dist = dis;
                while(i < 17 && dist > 0)
                {
                    proc_ranges[dist] = 0.0;
                    dist--;
                    i++;
                }
                i = 0;
                dist = dis;
                while(i < 17 && dist < 1080)
                {
                    proc_ranges[dist] = 0.0;
                    dist++;
                    i++;
                }

                //Find max length gap
                // this->find_max_gap(range);

                float inc = scan_msg.angle_increment;

                //Find the best point in the gap
                angle = this->find_best_point(proc_ranges, inc);
            }
            else
            {
                angle = 0.0;
            }
                
            //Publish drive message
            this->pid_control(angle);
        }

        void pid_control(double angle)
        {
            double velocity;
            ackermann_msgs::AckermannDriveStamped drive_msgs;

            //use kp, ki and kd to implement a PID controller for
            // angle = ((kd*error*error)+(kp*error)+ki)/((error*error*error)+((10+kd)*error*error)+((20+kp)*error)+ki);

            //calculating the velocity from the turning angles
            if(angle > 0.01)
            {
                if(angle < 0.174533) velocity = 1.5;
                else if(angle < 0.349) velocity = 1.0;
                else velocity = 0.5;
            }
            else if(angle < -0.01)
            {
                if(angle < -0.174533) velocity = -1.5;
                else if(angle < -0.349) velocity = -1.0;
                else velocity = -0.5;
            }
            else velocity = 1.5;

            // if(angle > -0.174533 && angle < 0) velocity = 1.5;
            // else if(angle < 0.174533 && angle > 0) velocity = -1.5;
            // else if (angle >= 0.174533 && angle < 0.349) velocity = 1.0;
            // else if(angle <=-0.174533 && angle >= -0.349) velocity = -1.0;
            // else velocity = 0.5;

            //publishing the data to the /nav topic 
            drive_msgs.header.stamp = ros::Time::now();
            drive_msgs.header.frame_id = "laser";
            drive_msgs.drive.steering_angle = angle;
            drive_msgs.drive.speed = velocity;
            drive_pub.publish(drive_msgs);
        }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "reactive_follow_gap");
    reactiveFollowGap rfg;
    ros::spin();
    return 0;
}