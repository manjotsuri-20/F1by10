#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <f1_pkg/pid_law.h>

#define car_width 0.2032 //metres
#define obstacle_thresh 1.5

#include <chrono>
#define P_RESET   "\033[0m"
#define P_BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
static std::map<std::string, std::chrono::steady_clock::time_point> logging_time_begin;
#define TIK(n) logging_time_begin[n] = std::chrono::steady_clock::now();
#define TOK(n) std::cout<<P_BOLDGREEN<<n<<" took "<<std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - logging_time_begin[n]).count() << " [µs]"<<P_RESET << std::endl; logging_time_begin.erase(n);


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

        pidLaw* pid_law;

    public:
        reactiveFollowGap();

        ~reactiveFollowGap();

        //Preprocess the LiDAR scan array. Expert implementation includes:
        //1.Setting each value to the mean over some window
        //2.Rejecting high values (eg. > 3m)
        //narrowing down the view of the lidar i.e. total readings divide by 5
        //and returning the shorter range
        std::vector<float> preprocess_lidar(std::vector<float> range);

        //finding the largest gap
        int find_max_gap_var(std::vector<float> range);

        // older version to find the gap
        int find_max_gap(std::vector<float> range);

        //Start_i & end_i are start and end indicies of max-gap range, respectively
        //Return index of best point in ranges
        //Naive: Choose the furthest point within ranges and go there

        //calculating the angle from the front vertical axis 
        //and then returning the angle
        double find_best_point(std::vector<float> range, float inc, int dis);

        void lidar_callback(const sensor_msgs::LaserScan &scan_msg);

        void pid_control(double angle);

};