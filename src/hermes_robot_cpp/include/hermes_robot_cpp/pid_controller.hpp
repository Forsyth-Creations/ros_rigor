#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>

class PIDController
{
public:
    PIDController(double p, double i, double d, double target,
                  double min_output = -2.0, double max_output = 2.0,
                  rclcpp::Logger logger = rclcpp::get_logger("PIDController"))
        : p_(p), i_(i), d_(d), target_(target),
          min_output_(min_output), max_output_(max_output),
          prev_error_(0.0), integral_(0.0), logger_(logger)
    {
    }

    double compute_move(double requested_value, double actual_value)
    {
        double error = requested_value - actual_value;
        integral_ += error;
        double derivative = error - prev_error_;
        double output = p_ * error + i_ * integral_ + d_ * derivative;
        prev_error_ = error;

        // Clamp the output within the allowed range
        // output = std::clamp(output, min_output_, max_output_);

        // Round to 3 decimal places
        return std::round(output * 1000.0) / 1000.0;
    }

private:
    double p_;
    double i_;
    double d_;
    double target_;
    double min_output_;
    double max_output_;
    double prev_error_;
    double integral_;
    rclcpp::Logger logger_;
};

#endif // PID_CONTROLLER_HPP
