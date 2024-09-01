#include <thread>
#include <mutex>
#include <signal.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <f1_pkg/pid_law.h>

bool signalFlag{false};

// for debugging and calculate time taken by functions
#include <chrono>
#define P_RESET   "\033[0m"
#define P_BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
static std::map<std::string, std::chrono::steady_clock::time_point> logging_time_begin;
#define TIK(n) logging_time_begin[n] = std::chrono::steady_clock::now();
#define TOK(n) std::cout<<P_BOLDGREEN<<n<<" took "<<std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - logging_time_begin[n]).count() << " [µs]"<<P_RESET << std::endl; logging_time_begin.erase(n);

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

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
        ros::Publisher vis_pub;
        ros::Publisher vis_gap_pub;
        ros::Subscriber lidar_sub;

        pidLaw* pid_law;

        int m_best_index = 0;
        double m_turnAngle = 0.0;
        std::string m_scanTopic;
        std::string m_debugScanTopic;
        std::string m_debugMarkerTopic;
        std::string m_debugGapTopic;
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
        bool m_real_gap_found{false};

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
        
        /**
         * @brief Create a publishers and subscribers object
         * 
         */
        void create_publishers_and_subscribers();

        /**
         * @brief Preprocess the LiDAR scan array. Expert implementation includes:
        1.Setting each value to the mean over some window
        2.Rejecting high values (eg. > 3m)
        narrowing down the view of the lidar i.e. total readings divide by 5
        and returning the shorter range
         * 
         */
        void preprocess_lidar();

        /**
         * @brief finding the largest gap
         * 
         */
        void find_max_gap();

        /**
         * @brief avoid the whole range of obstacles in the defined radius
         * 
         * @param index 
         * @param noOfReadings 
         */
        void avoid_whole_bubble(int index, int noOfReadings);

        /**
         * @brief Avoid nearest obstacles i.e.obstacles which are nearer than a threshould distance
         * 
         */
        void avoid_nearest_obstacles();

        /**
         * @brief to calculte disparity among the lidar readings in order to choose the best gap
         * 
         */
        void calculate_disparity();

        /**
         * @brief Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there

        calculating the angle from the front vertical axis 
        and then returning the angle
         * 
         */
        void find_best_point();

        /**
         * @brief debug processed scan
         * 
         */
        void publish_debug_scan();

        /**
         * @brief Visualise the best index chosen
         * 
         */
        void publish_marker();

        /**
         * @brief publish the gap which is chosen finally
         * 
         * @param begin 
         * @param end 
         */
        void publish_gap(int begin, int end);

        /**
         * @brief Process each LIDAR scan as per the Follow Gap algorithms 
        & publish an AckermannDriveStamped Message 
         * 
         * @param scan_msg 
         */
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