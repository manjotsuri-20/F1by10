#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <f1_pkg/pid_paramConfig.h>

class pidLaw
{
    private:
        double kp, ki, kd;
        ros::NodeHandle n_pid;
        double integral = 0.0, derivative = 0.0, prev_err = 0.0;

        // Create dynamic reconfigure server
        dynamic_reconfigure::Server<f1_pkg::pid_paramConfig> server;
        dynamic_reconfigure::Server<f1_pkg::pid_paramConfig>::CallbackType f;

    public:

        pidLaw(ros::NodeHandle n);

        ~pidLaw();

        double calculateError(double err);

        void reconfigureCallback(f1_pkg::pid_paramConfig &config, uint32_t level);
};