#include "f1_pkg/reactive_follow_gap.h"

reactiveFollowGap::reactiveFollowGap(ros::NodeHandle nh) : n_rea(nh)
{
    std::cout << "[REACTIVE FOLLOW GAP][INFO] Reactive Follow Gap Constructor called\n";

    is_running = true;

    pid_law = new pidLaw(n_rea);

    getParams();

    drive_pub = n_rea.advertise<ackermann_msgs::AckermannDriveStamped>(m_driveTopic, 1);
    debug_scan_pub = n_rea.advertise<sensor_msgs::LaserScan>(m_debugScanTopic, 1);
    lidar_sub = n_rea.subscribe(m_scanTopic, 1, &reactiveFollowGap::lidar_callback, this);

    rf = std::thread(&reactiveFollowGap::run, this);  
}

reactiveFollowGap::~reactiveFollowGap()
{
    stop();
    delete pid_law;
    std::cout << "[REACTIVE FOLLOW GAP][INFO] Reactive Follow Gap Destructor called\n";
}

void reactiveFollowGap::getParams()
{
    if(!n_rea.getParam("reactive_follow_gap/car_width", m_carWidth))
    {
        std::cerr << "Car Width parameter not found\n";
        std::exit(EXIT_FAILURE);
    }
    if(!n_rea.getParam("reactive_follow_gap/min_obstacle_threshould", m_minObstacleThreshould))
    {
        std::cerr << "Min Obstacle Threshould parameter not found\n";
        std::exit(EXIT_FAILURE);
    }
    if(!n_rea.getParam("reactive_follow_gap/max_obstacle_threshould", m_maxObstacleThreshould))
    {
        std::cerr << "Max Obstacle Threshould parameter not found\n";
        std::exit(EXIT_FAILURE);
    }
    if(!n_rea.getParam("reactive_follow_gap/scan_topic", m_scanTopic))
    {
        std::cerr << "Scan Topic parameter not found\n";
        std::exit(EXIT_FAILURE);
    }
    if(!n_rea.getParam("reactive_follow_gap/drive_topic", m_driveTopic))
    {
        std::cerr << "Drive topic parameter not found\n";
        std::exit(EXIT_FAILURE);
    }
    if(!n_rea.getParam("reactive_follow_gap/start_angle", m_startAngle))
    {
        std::cerr << "Start Angle parameter not found\n";
        std::exit(EXIT_FAILURE);
    }
    if(!n_rea.getParam("reactive_follow_gap/end_angle", m_endAngle))
    {
        std::cerr << "End Angle parameter not found\n";
        std::exit(EXIT_FAILURE);
    }
    if(!n_rea.getParam("reactive_follow_gap/debug", m_debug))
    {
        std::cerr << "Debug parameter not found\n";
        std::exit(EXIT_FAILURE);
    }
    if(!n_rea.getParam("reactive_follow_gap/debug_scan", m_debugScanTopic))
    {
        std::cerr << "Debug scan parameter not found\n";
        std::exit(EXIT_FAILURE);
    }
    if(!n_rea.getParam("reactive_follow_gap/detect_disparity_threshould", m_detectDisparityTreshould))
    {
        std::cerr << "Detect disparity threshould parameter not found\n";
        std::exit(EXIT_FAILURE);
    }
    if(!n_rea.getParam("reactive_follow_gap/max_speed", m_maxSpeed))
    {
        std::cerr << "Max Speed parameter not found\n";
        std::exit(EXIT_FAILURE);
    }
    if(!n_rea.getParam("reactive_follow_gap/min_gap_size", m_minGapSize))
    {
        std::cerr << "Minimum Gap Size parameter not found\n";
        std::exit(EXIT_FAILURE);
    }
}

void reactiveFollowGap::preprocess_lidar()
{
    //Preprocess the LiDAR scan array. Expert implementation includes:
    //1.Setting each value to the mean over some window

    m_lidarRangeProcessed.clear(); m_lidarRangeProcessed.resize(0);
    int start = (m_lidarNumberOfRays * m_startAngle) / 360; // m_lidarNumberOfRays / ((2 * M_PI) / ((m_startAngle * M_PI) / 180));
    int end = (m_lidarNumberOfRays * m_endAngle) / 360; // m_lidarNumberOfRays / ((2 * M_PI) / ((m_endAngle * M_PI) / 180));
    if(m_lidarRange.size() >= end - start + 1) //to avoid crash while accessing the values m_lidarRange
    {
        for(int i = start; i <= end ; i ++ )
        {
            m_lidarRangeProcessed.emplace_back(m_lidarRange[i] < m_maxObstacleThreshould ? m_lidarRange[i] : m_maxObstacleThreshould); // to have maximum range not so far
        }
    }
}

void reactiveFollowGap::find_max_gap()
{
    int start_ind = 0, last_ind = m_lidarRangeProcessed.size() - 1;
    bool enc = false;
    std::vector<std::pair<int, int>> gaps;
    int max_gap = INT_MIN;

    if( m_lidarRangeProcessed.size() <= 0 )
    {
        std::cout << "[REACTIVE FOLLOW GAP][INFO] Lidar Range Size Zero\n";
        return;
    }
    m_best_index = m_lidarRangeProcessed.size() / 2; //go straight if no gap found

    //creating the gaps when there is no 0 value and storing first and last 
    //index of those gaps in a vector
    for(int i = 0; i < m_lidarRangeProcessed.size(); i++)
    {
        if(m_lidarRangeProcessed[i] == 0 && enc == false)
        {
            last_ind = i - 1;
            enc = true;
            gaps.emplace_back(std::make_pair(start_ind, last_ind));
        }
        else if(enc == true && m_lidarRangeProcessed[i] != 0)
        {
            start_ind = i;
            enc = false;
        }
        else if(enc == false && i == m_lidarRangeProcessed.size() - 1)
        {
            last_ind = i;
            gaps.emplace_back(std::make_pair(start_ind, last_ind));
        }
    }


    if( m_debug )
    {
        if(gaps.size() == 0)
        {
            std::cout << "[REACTIVE FOLLOW GAP][INFO] No Gaps Found\n";
        }
    }

    //chosing the largest gap from all the gaps encountered
    for(auto i : gaps)
    {
        if((i.second - i.first) > m_minGapSize && (i.second - i.first) > max_gap)
        {
            max_gap = i.second - i.first; //update max_gap
            m_best_index = (i.first + i.second)/2;
        }
    }
}

void reactiveFollowGap::calculate_disparity()
{
    for(int i = 1; i < m_lidarRangeProcessed.size(); i++) // starting from 1 because subtracting the previous index
    {
        int j = 0, val = 0;
        int numberOfReadings = m_carWidth / (m_lidarRangeProcessed[i] * m_lidarAngleIncrement);
        if( m_lidarRangeProcessed[i] - m_lidarRangeProcessed[i - 1] > m_detectDisparityTreshould )
        {
            while(j < numberOfReadings)
            {
                if( i >= m_lidarRangeProcessed.size()) //to end the loop if number of reading have become greater than the size.
                {
                    return;
                }
                if( j == 0 )
                {
                    val = m_lidarRangeProcessed[i - 1];
                }
                m_lidarRangeProcessed[i] = val + 0.2;
                j++; i++;   
            }
        }
        else if( m_lidarRangeProcessed[i - 1] - m_lidarRangeProcessed[i] > m_detectDisparityTreshould )
        {
            while (j < numberOfReadings)
            {
                if( i < 1 )
                {
                    return;
                }
                if( j == 0 )
                {
                    val = m_lidarRangeProcessed[i];
                }
                m_lidarRangeProcessed[i - 1] = val + 0.2;
                j++; i--;
            }
            i += numberOfReadings;
        }
    }
}

void reactiveFollowGap::find_best_point()
{
    //Start_i & end_i are start and end indicies of max-gap range, respectively
    //Return index of best point in ranges
    //Naive: Choose the furthest point within ranges and go there

    //calculating the angle from the front vertical axis 
    //and then returning the angle
    // auto dis = std::distance(range.begin(), std::max_element(range.begin(), range.end()));
    int diff = m_best_index - m_lidarRangeProcessed.size() / 2;
    m_turnAngle =  double(diff * m_lidarAngleIncrement);
}

void reactiveFollowGap::avoid_whole_bubble(int index, int noOfReadings)
{
    //setting the nearby values of nearest point to zero so as to avoid the obstacle
    int i = 0, temp_index = index;
    if( m_lidarRangeProcessed.size() > temp_index )
    {
        while(i < noOfReadings && temp_index > 0)
        {
            m_lidarRangeProcessed[temp_index] = 0.0;
            temp_index--;
            i++;
        }
    }
    i = 0;
    temp_index = index;
    if( m_lidarRangeProcessed.size() > (temp_index + noOfReadings) )
    {
        while(i < noOfReadings && temp_index < m_lidarNumberOfRays)
        {
            m_lidarRangeProcessed[temp_index] = 0.0;
            temp_index++;
            i++;
        } 
    }
}

void reactiveFollowGap::avoid_nearest_obstacles()
{
    for( int i = 0; i < m_lidarRangeProcessed.size(); i++ )
    {
        //checking whether the obstacle is under the threshould or outside the
        //threshould region. If inside the threshould then avoiding the obstacle
        //else moving straight
        if( m_lidarRangeProcessed[i] < m_minObstacleThreshould )
        {
            int numberOfReadings = (m_carWidth / m_lidarRangeProcessed[i]) / m_lidarAngleIncrement;
            avoid_whole_bubble(i, numberOfReadings);
            i += numberOfReadings; //skipping because they have already been made zero because they were inside obstacle bubble
        }
    }
}

void reactiveFollowGap::pid_control()
{
    double velocity;
    ackermann_msgs::AckermannDriveStamped drive_msgs;

    //angle on right is negative and on left is positive
    
    //use kp, ki and kd to implement a PID controller for
    // angle = ((kd*angle*angle)+(kp*angle)+ki)/((angle*angle*angle)+((10+kd)*angle*angle)+((20+kp)*angle)+ki);
    // m_turnAngle = pid_law->calculateError(m_turnAngle);


    // velocity = (m_maxSpeed / (m_turnAngle + 1.0)); // linear function for increasing and decreasing velocity

    //calculating the velocity from the turning angles
    if(fabs(m_turnAngle) < 0.08)
    {
        velocity = m_maxSpeed;
    }
    else if(fabs(m_turnAngle) < 0.349) 
    {
        velocity = 3.5;
    }
    else 
    {
        velocity = 2.0;
    }

    //publishing the data to the /nav topic 
    drive_msgs.header.stamp = ros::Time::now();
    drive_msgs.header.frame_id = "laser";
    drive_msgs.drive.steering_angle = m_turnAngle;
    drive_msgs.drive.speed = velocity;
    drive_pub.publish(drive_msgs);
}

void reactiveFollowGap::publish_debug_scan()
{
    // Publish the laser message
    sensor_msgs::LaserScan scan_msg;
    scan_msg.header.stamp = ros::Time::now();
    scan_msg.header.frame_id = "laser";
    scan_msg.angle_min = (m_startAngle * M_PI / 180) + M_PI;
    scan_msg.angle_max = (m_endAngle * M_PI / 180) + M_PI;
    scan_msg.angle_increment = m_lidarAngleIncrement;
    scan_msg.range_max = 100;
    scan_msg.ranges = m_lidarRangeProcessed;
    scan_msg.intensities = m_lidarRangeProcessed;

    debug_scan_pub.publish(scan_msg);
}

void reactiveFollowGap::lidar_callback(const sensor_msgs::LaserScan &scan_msg)
{
    // std::unique_lock<std::mutex> lock(m_lidarMutex);
    m_lidarRange = scan_msg.ranges;
    m_lidarAngleIncrement = scan_msg.angle_increment;
    m_lidarNumberOfRays = (scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment;    
    callback_running = true;
}

void reactiveFollowGap::publish_zero_velocity()
{
    std::cout << "[REACTIVE FOLLOW GAP][INFO] Publishing Zero Velocity\n";

    //publish zero velocity
    ackermann_msgs::AckermannDriveStamped drive_msgs;
    drive_msgs.header.stamp = ros::Time::now();
    drive_msgs.header.frame_id = "laser";
    drive_msgs.drive.steering_angle = 0.0;
    drive_msgs.drive.speed = 0.0;
    drive_pub.publish(drive_msgs);
}

void reactiveFollowGap::run()
{
    while(!signalFlag && is_running)
    {
        // TIK("loop time")
        // std::unique_lock<std::mutex> lock(m_lidarMutex);

        if( callback_running )
        {
            preprocess_lidar(); //Preprocessed ranges
        
            // calculate_disparity(); not using it currently

            avoid_nearest_obstacles();
            
            if( m_debug )
            {
                publish_debug_scan();
            } 

            find_max_gap(); //Find max length gap

            find_best_point();
                
            pid_control(); //Publish drive message
        }

        // lock.unlock();

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        // TOK("loop time")
    }
}

void reactiveFollowGap::stop()
{
    is_running = false;
    if ( rf.joinable() ) 
    {
        rf.join();  // Wait for the thread to finish
    }
    publish_zero_velocity();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reactive_follow_gap");
    ros::NodeHandle nh;

    reactiveFollowGap rfg(nh);

	signal(SIGINT, signalInterrupt);
    while(ros::ok() && !signalFlag)
    {
        ros::spinOnce();
    }
    return 0;
}