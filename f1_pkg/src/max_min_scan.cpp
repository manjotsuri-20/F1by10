#include <ros/ros.h>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <f1_pkg/scan_range.h>

class Max_min
{
    private:
    //NodeHandle for ROS
    ros::NodeHandle n;

    //Publishers
    ros::Publisher closest_pub;
    ros::Publisher farthest_pub;
    ros::Publisher both_pub;

    //Subscribers
    ros::Subscriber scan_msg;

    std_msgs::Float64 val;
    f1_pkg::scan_range both;


    public:
    Max_min()
    {
        closest_pub = n.advertise<std_msgs::Float64>("/closest_point", 1);
        farthest_pub = n.advertise<std_msgs::Float64>("/farthest_point", 1);
        both_pub = n.advertise<f1_pkg::scan_range>("/scan_range", 1);
        scan_msg = n.subscribe("/scan", 1, &Max_min::scan_callback, this);
    }

    void scan_callback(const sensor_msgs::LaserScan &msg)
    {
        auto data_scan = msg.ranges;

        for(auto i : data_scan)
        {
            if(std::isinf(i) || std::isnan(i))
            {
                data_scan.erase(data_scan.begin()+i);
            }
        }

        val.data=*std::min_element(data_scan.begin(), data_scan.end());
        closest_pub.publish(val);
        ROS_INFO("closest point = %f", val.data);
        val.data=*std::max_element(data_scan.begin(), data_scan.end());
        farthest_pub.publish(val);
        ROS_INFO("farthest point = %f", val.data);
        both.max = *std::max_element(data_scan.begin(), data_scan.end());
        both.min = *std::min_element(data_scan.begin(), data_scan.end());
        both_pub.publish(both);
        ROS_INFO("Scan Range published");
    }
    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "max_min");
    Max_min mm;
    ros::spin();
    return 0;
}