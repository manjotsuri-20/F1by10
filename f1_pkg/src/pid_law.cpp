#include <f1_pkg/pid_law.h>

pidLaw::pidLaw(ros::NodeHandle n) : n_pid(n)
{
    if(!n_pid.getParam("/reactive_follow_gap/kp", kp))
    {
        std::cout << "Kp value not defined in parameters\n";
        std::exit(EXIT_FAILURE);
    }
    if(!n_pid.getParam("/reactive_follow_gap/kd", kd))
    {
        std::cout << "Kd value not defined in parameters\n";
        std::exit(EXIT_FAILURE);
    }
    if(!n_pid.getParam("/reactive_follow_gap/ki", ki))
    {
        std::cout << "ki value not defined in parameters\n";
        std::exit(EXIT_FAILURE);
    }

    // Bind callback function
    f = boost::bind(&pidLaw::reconfigureCallback, this, _1, _2);
    server.setCallback(f);
}

pidLaw::~pidLaw(){}

double pidLaw::calculateError(double err)
{
    integral += err; // Accumulate integral term

    derivative = err - prev_err; // Calculate derivative term
    prev_err = err; // Update previous error

    double output = kp * err + ki * integral + kd * derivative; // Calculate PID output

    return output;
}

void pidLaw::reconfigureCallback(f1_pkg::pid_paramConfig &config, uint32_t level)
{
    ROS_INFO("Parameters Updated");
    std::cout << "value of kp updated: "<< config.kp << std::endl;
    std::cout << "value of ki updated: "<< config.ki << std::endl;
    std::cout << "value of kd updated: "<< config.kd << std::endl;

}