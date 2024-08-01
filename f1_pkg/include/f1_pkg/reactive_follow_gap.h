#include <thread>
#include <mutex>
#include <signal.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <f1_pkg/pid_law.h>

bool signalFlag{false};

//for debugging and calculate time taken by functions
// #include <chrono>
// #define P_RESET   "\033[0m"
// #define P_BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
// static std::map<std::string, std::chrono::steady_clock::time_point> logging_time_begin;
// #define TIK(n) logging_time_begin[n] = std::chrono::steady_clock::now();
// #define TOK(n) std::cout<<P_BOLDGREEN<<n<<" took "<<std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - logging_time_begin[n]).count() << " [µs]"<<P_RESET << std::endl; logging_time_begin.erase(n);

/*
    Lidar Readings array indices
                540
            810     270
                 0
*/

/**
 * @brief function to detect user interrupts
*/
void signalInterrupt(int sig)
{
	if (sig == SIGINT)
	{
		ROS_WARN("CTRL+C detected, stop");
		signalFlag = true;
	}
}

class reactiveFollowGap
{
    private:
        ros::NodeHandle n_rea;
        ros::Publisher drive_pub;
        ros::Publisher debug_scan_pub;
        ros::Subscriber lidar_sub;

        pidLaw* pid_law;

        int m_best_index = 0;
        double m_turnAngle = 0.0;
        std::string m_scanTopic;
        std::string m_debugScanTopic;
        std::string m_driveTopic;

        //lidar data
        float m_lidarAngleIncrement = 0.0;
        std::vector<float> m_lidarRange, m_lidarRangeProcessed;
        int m_lidarNumberOfRays = 0.0;

        //Params
        int m_minGapSize = 0.0;
        float m_carWidth = 0.0; //metres
        float m_minObstacleThreshould = 0.0, m_maxObstacleThreshould = 0.0; //metres
        float m_startAngle = 0.0, m_endAngle = 0.0, m_maxSpeed = 0.0;
        float m_detectDisparityTreshould = 0.0;
        bool m_debug = false;

        std::thread rf;

        std::atomic_bool is_running{false};

        std::mutex m_lidarMutex;

        bool callback_running = false;

    public:
        reactiveFollowGap(ros::NodeHandle);

        ~reactiveFollowGap();

        /**
         * @brief load all the parameters required by reactive follow gap node
        */
        void getParams();

        //Preprocess the LiDAR scan array. Expert implementation includes:
        //1.Setting each value to the mean over some window
        //2.Rejecting high values (eg. > 3m)
        //narrowing down the view of the lidar i.e. total readings divide by 5
        //and returning the shorter range
        void preprocess_lidar();

        //finding the largest gap
        void find_max_gap();

        void avoid_whole_bubble(int index, int noOfReadings);

        void avoid_nearest_obstacles();

        // to calculte disparity among the lidar readings in order to choose the best gap
        void calculate_disparity();

        //Start_i & end_i are start and end indicies of max-gap range, respectively
        //Return index of best point in ranges
        //Naive: Choose the furthest point within ranges and go there

        //calculating the angle from the front vertical axis 
        //and then returning the angle
        void find_best_point();

        //debug processed scan
        void publish_debug_scan();

        //Process each LIDAR scan as per the Follow Gap algorithms 
        //& publish an AckermannDriveStamped Message 
        void lidar_callback(const sensor_msgs::LaserScan &scan_msg);

        /**
         * @brief take action based on the PID values
        */
        void pid_control();

        /**
         * @brief publish zero velocity whenever required
        */
        void publish_zero_velocity();

        /**
         * @brief main loop of the program
        */
        void run();

        /**
         * @brief exit all processes and thread
        */
        void stop();

};