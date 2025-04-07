#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hermes_robot_cpp/pid_controller.hpp"

using std::placeholders::_1;

class Module : public rclcpp::Node
{
public:
    Module(const std::string& name) : Node(name)
    {
        pivot_error_ = 0.0;
        actual_wheel_speed_ = 0.0;
        rqst_wheel_speed_ = 0.0;
        drive_wheel_position_ = 0.0;
        target_wheel_position_ = 0.0;
        rqst_pivot_angle_ = 0.0;
        actual_pivot_angle_ = 0.0;
        commanded_wheel_speed_ = 0.0;
        commanded_pivot_position_ = 0.0;
        limit_switch_triggered_ = false;

        warning_flag_for_dime_ = false;

        module_name_ = this->declare_parameter<std::string>("module_name", "module_default");
        mock_encoder_values_ = this->declare_parameter<bool>("mock_encoder_values", true);
        invert_drive_motor_ = this->declare_parameter<bool>("invert_drive_motor", false);
        invert_pivot_motor_ = this->declare_parameter<bool>("invert_pivot_motor", false);
        odom_frequency_ = this->declare_parameter<double>("odom_frequency", 10.0);
        enable_logging_ = this->declare_parameter<bool>("enable_logging", false);



        if (invert_drive_motor_)
        {
            RCLCPP_INFO(this->get_logger(), "\033[32mInverting Drive Motor for %s\033[39m", module_name_.c_str());
        }

        command_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/" + module_name_ + "/command", 10, std::bind(&Module::command_callback, this, _1));
        encoder_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/" + module_name_ + "/encoders", 10);
        limit_switch_pub_ = this->create_publisher<std_msgs::msg::Float64>("/" + module_name_ + "/limit_switch", 10);
        pivot_position_pub_ = this->create_publisher<std_msgs::msg::Float64>("/" + module_name_ + "/pivot_position", 10);
        drive_motor_pub_ = this->create_publisher<std_msgs::msg::Float64>("/" + module_name_ + "/wheel_position", 10);
        speed_motor_pub_ = this->create_publisher<std_msgs::msg::Float64>("/" + module_name_ + "/wheel_speed", 10);
        pivot_error_pub_ = this->create_publisher<std_msgs::msg::Float64>("/" + module_name_ + "/pivot_error", 10);

        error_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Module::compute_pivot_error, this));
        update_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / odom_frequency_)), std::bind(&Module::update_position, this));

        drive_pid_controller_ = std::make_unique<PIDController>(0.1, 0.4, 0.01, rqst_wheel_speed_, -2.0, 2.0, this->get_logger());
        pivot_pid_controller_ = std::make_unique<PIDController>(0.1, 0.1, 0.01, rqst_pivot_angle_, -6.28, 6.28, this->get_logger());

        RCLCPP_INFO(this->get_logger(), "\033[34m Module Ready: %s\033[39m", module_name_.c_str());
    }

private:
    std::string module_name_;
    bool mock_encoder_values_;
    bool invert_drive_motor_;
    bool invert_pivot_motor_;
    bool warning_flag_for_dime_;
    bool limit_switch_triggered_;

    double odom_frequency_;
    double pivot_error_;
    double actual_wheel_speed_;
    double rqst_wheel_speed_;
    double drive_wheel_position_;
    double target_wheel_position_;
    double rqst_pivot_angle_;
    double actual_pivot_angle_;
    double commanded_wheel_speed_;
    double commanded_pivot_position_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr encoder_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr limit_switch_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pivot_position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr drive_motor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_motor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pivot_error_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;

    rclcpp::TimerBase::SharedPtr error_timer_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    std::unique_ptr<PIDController> drive_pid_controller_;
    std::unique_ptr<PIDController> pivot_pid_controller_;

    bool enable_logging_;

    void command_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {


        set_rqst_pivot_angle(msg->angular.z);
        compute_pivot_error();
        set_rqst_wheel_speed(msg->linear.x);

        // // Log that the command was received
        // RCLCPP_INFO(this->get_logger(), "Command received for module %s: linear.x=%.2f, angular.z=%.2f",
        //             module_name_.c_str(), msg->linear.x, msg->angular.z);
    }

    void compute_pivot_error()
    {
        pivot_error_ = rqst_pivot_angle_ - actual_pivot_angle_;
        std_msgs::msg::Float64 error_msg;
        error_msg.data = pivot_error_;
        pivot_error_pub_->publish(error_msg);
    }

    void set_rqst_wheel_speed(double speed)
    {
        if (pivot_error_ > 0.1)
            rqst_wheel_speed_ *= 0.95;
        else
            rqst_wheel_speed_ = speed;
    }

    void set_rqst_pivot_angle(double angle)
    {
        rqst_pivot_angle_ = angle;

        // Print the requested pivot angle
        // RCLCPP_INFO(this->get_logger(), "Requested pivot angle for module %s: %.2f", module_name_.c_str(), rqst_pivot_angle_);
    }

    void update_encoder_values()
    {
        if (!mock_encoder_values_)
        {
            throw std::runtime_error("Actual encoder values not implemented yet");
        }
        else
        {
            actual_wheel_speed_ = commanded_wheel_speed_;
            actual_pivot_angle_ = commanded_pivot_position_;
        }
    }

    void update_limit_switch()
    {
        if (!mock_encoder_values_)
        {
            throw std::runtime_error("Actual limit switch state not implemented yet");
        }
        else
        {
            limit_switch_triggered_ = false;
        }
    }

    void update_position()
    {
        update_encoder_values();
        update_limit_switch();

        double drive_output = drive_pid_controller_->compute_move(rqst_wheel_speed_, actual_wheel_speed_);
        // double drive_output = 0;

        if (enable_logging_)
        {
            RCLCPP_INFO(this->get_logger(), "Drive output for module %s: %.2f when I requested %.2f",
                        module_name_.c_str(), drive_output, rqst_wheel_speed_);
        }


        // Publish the speed and drive motor values
        std_msgs::msg::Float64 speed_msg;
        speed_msg.data = invert_drive_motor_ ? -drive_output : drive_output;
        speed_motor_pub_->publish(speed_msg);

        drive_wheel_position_ += invert_drive_motor_ ? -drive_output : drive_output;
        std_msgs::msg::Float64 drive_msg;
        drive_msg.data = drive_wheel_position_;
        drive_motor_pub_->publish(drive_msg);

        // print out the drive wheel position
        if (enable_logging_)
        {
            RCLCPP_INFO(this->get_logger(), "Drive wheel position for module %s: %.2f", module_name_.c_str(), drive_wheel_position_);
        }

        double pivot_output = pivot_pid_controller_->compute_move(rqst_pivot_angle_, actual_pivot_angle_);
        std_msgs::msg::Float64 pivot_msg;

        // if (rqst_wheel_speed_ != 0.0)
        // {
        //     pivot_msg.data = invert_pivot_motor_ ? -pivot_output : pivot_output;
        // }
        // else
        // {
        //     // print that the pivot is not moving
        //     RCLCPP_INFO(this->get_logger(), "You have commanded no velocity, so the pivot is not moving");
        //     pivot_msg.data = commanded_pivot_position_;
        // }

        pivot_msg.data = 0;

        pivot_position_pub_->publish(pivot_msg);

        if (mock_encoder_values_)
        {
            commanded_wheel_speed_ = drive_output;
            commanded_pivot_position_ = pivot_output;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Module>("hermes_swerve_module");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
