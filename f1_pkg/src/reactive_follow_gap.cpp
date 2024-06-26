#include "f1_pkg/reactive_follow_gap.h"

reactiveFollowGap::reactiveFollowGap()
{
    n_rea = ros::NodeHandle();

    pid_law = new pidLaw(n_rea);

    //Topics and Subscribers, Publishers
    lidarscan_topic = "/scan";
    drive_topic = "/nav";

    drive_pub = n_rea.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
    lidar_sub = n_rea.subscribe(lidarscan_topic, 1, &reactiveFollowGap::lidar_callback, this);

}

reactiveFollowGap::~reactiveFollowGap()
{
    delete pid_law;
}

std::vector<float> reactiveFollowGap::preprocess_lidar(std::vector<float> range)
{
    //Preprocess the LiDAR scan array. Expert implementation includes:
    //1.Setting each value to the mean over some window
    //2.Rejecting high values (eg. > 3m)

    //narrowing down the view of the lidar i.e. total readings divide by 5
    //and returning the shorter range
    std::vector<float> for_range;
    int start = 270, end = 810;
    for(int i = start; i < end ; i ++ )
    {
        for_range.emplace_back(range[i]);
    }
    return for_range;
}

int reactiveFollowGap::find_max_gap_var(std::vector<float> range)
{
    int start_ind = 0, last_ind = range.size() - 1;
    bool enc = false;
    std::vector<std::pair<int, int>> gaps;

    //creating the gaps when there is no 0 value and storing first and last 
    //index of those gaps in a vector
    for(int i = 0; i < range.size(); i++)
    {
        if(range[i] == 0 && enc == false)
        {
            last_ind = i - 1;
            enc = true;
            gaps.emplace_back(std::make_pair(start_ind, last_ind));
        }
        else if(enc == true && range[i] != 0)
        {
            start_ind = i;
            enc = false;
        }
        else if(enc == false && i == range.size() - 1)
        {
            last_ind = i;
            gaps.emplace_back(std::make_pair(start_ind, last_ind));
        }
    }

    int chosen_ind = range.size()/2;
    int min_gap = 4;

    //chosing the largest gap from all the gaps encountered
    for(auto i : gaps)
    {
        if(i.second - i.first > min_gap)
        {
            min_gap = i.second - i.first;
            chosen_ind = (i.first + i.second)/2;
        }
    }

    return chosen_ind;
}

int reactiveFollowGap::find_max_gap(std::vector<float> range)
{
    //Return the start and end index of the max gap in free_space_ranges
    
    std::map<int, int> max;
    std::vector<int> max_ind, not_chosen;
    
    int prev = 0;
    int max_gap = 4;
    int chosen_ind = 0;

    //checking the max_gap on the basis of largest gap in the array around the local maxima

    //checking the local maximas
    //and storing their indices in the vector
    for(int i = 1; i < range.size() - 1; i++)
    {
        if(range[i] > range[i-1] && range[i] > range[i+1])
        {
            max_ind.emplace_back(i);                   
        }
    }

    //creating a map with the index and the gap at that index
    for(int i = 0; i < max_ind.size(); i++)
    {
        if(i == 0)
        {
            max.insert(std::pair<int, int>(max_ind[i], max_ind[i+1]));
            prev = max_ind[i];
        }
        else if(i == max_ind.size() - 1)
        {
            max.insert(std::pair<int, int>(max_ind[i], range.size() - prev));
        }
        else
        {
            max.insert(std::pair<int, int>(max_ind[i], max_ind[i+1] - prev));
            prev = max_ind[i];
        }
    }

    //removing the gap having zeroes.
    int prev_ind = 0;
    for(auto i : max)
    {   
        if(i.second > max_gap)
        {
            for(int j = prev_ind; j <= prev_ind + i.second; j++)
            {
                if(range[j] == 0) // If the element is 0 it means the obstacle is very near so it needs to be removed
                {
                    not_chosen.emplace_back(i.first);
                    // max.erase(i.first);
                    break;
                }
            }
        }
        prev_ind = i.first;
    }

    //checking the largest gap
    int prev_i = 0;
    for(auto i : max)
    {
        //checking if the index to be selected is resisted to be selected because that index contains zero
        auto it = std::find(not_chosen.begin(), not_chosen.end(), i.first);
        if(i.second > max_gap) 
        {
            if(it != not_chosen.end() && *it == i.first)
            {
                continue;
            }
            max_gap = i.second;
            chosen_ind = prev_i + i.second/2;
        }
        prev_i = i.first;
    }

    return chosen_ind;

}

double reactiveFollowGap::find_best_point(std::vector<float> range, float inc, int dis)
{
    //Start_i & end_i are start and end indicies of max-gap range, respectively
    //Return index of best point in ranges
    //Naive: Choose the furthest point within ranges and go there

    //calculating the angle from the front vertical axis 
    //and then returning the angle
    // auto dis = std::distance(range.begin(), std::max_element(range.begin(), range.end()));
    int diff = dis - range.size()/2;
    return double(diff*inc);
}

void reactiveFollowGap::lidar_callback(const sensor_msgs::LaserScan &scan_msg)
{
    //Process each LIDAR scan as per the Follow Gap algorithms 
    //& publish an AckermannDriveStamped Message 

    TIK("loop time")
    double angle = 0.0;
    auto range = scan_msg.ranges;

    //Preprocessed ranges
    auto req_range = this->preprocess_lidar(range);

    //Find Closest point to lidar
    auto dis = std::distance(req_range.begin(), std::min_element(req_range.begin(), req_range.end()));

    //checking whether the obstacle is under the threshould or outside the
    //threshould region. If inside the threshould then avoiding the obstacle
    //else moving straight
    if(req_range[dis] < obstacle_thresh)
    {
        //Eliminate all points inside 'bubble' (set them to zero)
        int i = 0;
        int dist = dis;

        //setting the nearby values of nearest point to zero so as to avoid the obstacle
        while(i < 17 && dist > 0)
        {
            req_range[dist] = 0.0;
            dist--;
            i++;
        }
        i = 0;
        dist = dis;
        while(i < 17 && dist < 1080)
        {
            req_range[dist] = 0.0;
            dist++;
            i++;
        }

        float inc = scan_msg.angle_increment;

        //Find max length gap
        // auto index = this->find_max_gap(req_range);
        TIK("find_gap")
        auto index = this->find_max_gap_var(req_range);
        TOK("find_gap")

        //Find the best point in the gap
        angle = this->find_best_point(req_range, inc, index);
    }
    else
    {
        angle = 0.0;
    }
        
    //Publish drive message
    this->pid_control(angle);
    TOK("loop time")
}

void reactiveFollowGap::pid_control(double angle)
{
    double velocity;
    ackermann_msgs::AckermannDriveStamped drive_msgs;

    //angle on right is negative and on left is positive
    
    //use kp, ki and kd to implement a PID controller for
    // angle = ((kd*angle*angle)+(kp*angle)+ki)/((angle*angle*angle)+((10+kd)*angle*angle)+((20+kp)*angle)+ki);
    angle = pid_law->calculateError(angle);

    //calculating the velocity from the turning angles
    if(angle < 0.174533 && angle > -0.174533) velocity = 1.5;
    else if(angle < 0.349 && angle > -0.349) velocity = 1.0;
    else velocity = 0.5;

    //publishing the data to the /nav topic 
    drive_msgs.header.stamp = ros::Time::now();
    drive_msgs.header.frame_id = "laser";
    drive_msgs.drive.steering_angle = angle;
    drive_msgs.drive.speed = velocity;
    drive_pub.publish(drive_msgs);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "reactive_follow_gap");
    reactiveFollowGap rfg;
    ros::spin();
    return 0;
}