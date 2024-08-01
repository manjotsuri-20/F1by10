#include <f1_pkg/pid_law.h>

pidLaw::pidLaw(ros::NodeHandle n) : n_pid(n)
{
    std::cout << "[PID LAW][INFO] PID Law Constructor called\n";
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

pidLaw::~pidLaw()
{
    std::cout << "[PID LAW][INFO] PID Law Destructor called\n";
}

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
    std::cout << "[PID LAW][INFO] Parameters Updated\n";
    std::cout << "[PID LAW][INFO] value of kp updated: "<< config.kp << std::endl;
    std::cout << "[PID LAW][INFO] value of ki updated: "<< config.ki << std::endl;
    std::cout << "[PID LAW][INFO] value of kd updated: "<< config.kd << std::endl;

}