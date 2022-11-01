// This file is based on work by Franz Pucher, Diffbot, (2020), GitHub repository, https://github.com/fjp/diffbot
#include "rclcpp/duration.hpp"
#include <control_toolbox/pid.hpp>
// Dynamic reconfigure
// #include "hoverboard_driver/HoverboardConfig.h"
#include <boost/thread/mutex.hpp>

class PID : public control_toolbox::Pid
{
    public:
        /**
         * @brief Construct a new PID object
         * 
         * @param p The proportional gain.
         * @param i The integral gain.
         * @param d The derivative gain.
         * @param i_max The max integral windup.
         * @param i_min The min integral windup.
         * @param out_min The min computed output.
         * @param out_max The max computed output.
         */
        PID(double p=0.0, double i=0.0, double d=0.0, double i_max=0.0, double i_min=0.0, bool antiwindup=false, double out_max=0.0, double out_min=0.0);

        /**
         * @brief Initialize the 
         * 
         * @param nh ROS node handle with possible namespace describing the purpose of the PID controller.
         * @param kF The feed forward gain.
         * @param kP The proportional gain.
         * @param kI The integral gain.
         * @param kD The derivative gain.
         * @param i_max The max integral windup.
         * @param i_min The min integral windup.
         * @param antiwindup 
         * @param out_min The min computed output.
         * @param out_max The max computed output.
         */
        void init(rclcpp::Node* node, double f, double p, double i, double d, double i_max, double i_min, bool antiwindup, double out_max, double out_min);

        /**
         * @brief Compute PID output value from error using process value, set point and time period
         * 
         * @param measured_value The process value measured by sensors.
         * @param setpoint The desired target value.
         * @param dt The delta time or period since the last call.
         * @return double Computed PID output value.
         */
        double operator()(const double &measured_value, const double &setpoint, const rclcpp::Duration &dt);

        /**
         * @brief Get the FPID parameters
         * 
         * @param f The feed forward gain.
         * @param p The proportional gain.
         * @param i The integral gain.
         * @param d The derivative gain.
         * @param i_max The max integral windup.
         * @param i_min The min integral windup.
         * @param antiwindup Enable or disable antiwindup check.
         */
        void getParameters(double &f, double &p, double &i, double &d, double &i_max, double &i_min);
        void getParameters(double &f, double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);

        /**
         * @brief Set the Parameters using the Gains object of control_toolbox::Pid
         * 
         * @param f The feed forward gain.
         * @param p The proportional gain.
         * @param i The integral gain.
         * @param d The derivative gain.
         * @param i_max The max integral windup.
         * @param i_min The min integral windup.
         * @param antiwindup Enable or disable antiwindup check.
         */
        void setParameters(double f, double p, double i, double d, double i_max, double i_min, bool antiwindup = false);

        /**
         * @brief Set the Output Limits of the PID controller
         * 
         * @param out_min 
         * @param out_max 
         */
        void setOutputLimits(double out_min, double out_max);

        /**
         * @brief Clam given value to upper and lower limits.
         * 
         * @param value Input value that's possibly clamped.
         * @param lower_limit Lower limit which the value must not exceed.
         * @param upper_limit Upper limit which the value must not exceed.
         * @return double Clamped value to range in between [lower_limit, upper_limit].
         */
        double clamp(const double& value, const double& lower_limit, const double& upper_limit);

        /**
         * @brief Get the current error.
         * 
         * @return double The current error computed from the measured and target value.
         */
        inline double getError() const { return error_; };

    private:
        double f_;
        double error_;
        double out_min_;
        double out_max_;
        rclcpp::Node* node_;
};
