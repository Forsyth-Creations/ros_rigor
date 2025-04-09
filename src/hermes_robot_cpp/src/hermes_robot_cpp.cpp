#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "hermes_robot_cpp/module_kinematics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class HermesRobotNode : public rclcpp::Node
{
public:
    HermesRobotNode()
        : Node("hermes_robot_node"), odom_frequency_(30.0), fatal_flag_(false)
    {
        this->declare_parameter("module_topic_prefixes", "swerve_a,swerve_b,swerve_c,swerve_d");
        this->get_parameter("module_topic_prefixes", module_prefix_string_);
        split_module_prefixes(module_prefix_string_);


        for (const auto &module : module_prefixes_)
        {
            drive_publishers_.push_back(
                this->create_publisher<geometry_msgs::msg::Twist>("/" + module + "/command", 10));

            error_subs_.push_back(
                this->create_subscription<std_msgs::msg::Float64>(
                    "/" + module + "/pivot_error", 10,
                    [this, module](const std_msgs::msg::Float64::SharedPtr msg)
                    {
                        module_errors_[module] = msg->data;
                    }));

            wheel_position_subs_.push_back(
                this->create_subscription<std_msgs::msg::Float64>(
                    "/" + module + "/wheel_position", 10,
                    [this, module](const std_msgs::msg::Float64::SharedPtr msg)
                    {
                        wheel_positions_[module] = msg->data;
                    }));

            pivot_angle_subs_.push_back(
                this->create_subscription<std_msgs::msg::Float64>(
                    "/" + module + "/pivot_position", 10,
                    [this, module](const std_msgs::msg::Float64::SharedPtr msg)
                    {
                        pivot_angles_[module] = msg->data;

                    }));
        }

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/hermes/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                cmd_vel_callback(msg);
            });

        // Initialize wheel positions and pivot angles and the module errors
        for (const auto &module : module_prefixes_)
        {
            wheel_positions_[module] = 0.0;
            pivot_angles_[module] = 0.0;
            module_errors_[module] = 0.0;
        }

        // intialize the robot positions
        robot_positions_ = {{"x_pos", 0.0}, {"y_pos", 0.0}, {"angular_z", 0.0}};

        // Create a publisher for the robot angle
        robot_angle_pub_ = this->create_publisher<std_msgs::msg::Float64>("/hermes/robot_angle", 10);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/hermes/imu", 10,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg)
            {
                imu_data_ = *msg;
                // robot_positions_["angular_z"] = msg->orientation.z;
                // Convert quaternion to Euler angles (yaw, pitch, roll)
                tf2::Quaternion quat;
                tf2::fromMsg(msg->orientation, quat);
                double roll, pitch, yaw;
                tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

                // Update the robot's angular position (yaw)
                robot_positions_["angular_z"] = yaw;

                // publish the robot angle
                std_msgs::msg::Float64 angle_msg;
                angle_msg.data = yaw;
                robot_angle_pub_->publish(angle_msg);

                // Determine if I have fallen over
                if (std::abs(roll) > 1.0 || std::abs(pitch) > 1.0)
                {
                    RCLCPP_ERROR(this->get_logger(), "\033[1;31mRobot has fallen over!\033[0m");
                    fatal_flag_ = true;
                }
                else
                {
                    fatal_flag_ = false;
                }

            });

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        odom_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / odom_frequency_)),
            std::bind(&HermesRobotNode::update_odometry, this));

        // -------------- Create the ModuleKinematics objects --------------


        double spacing = 0.1524;
        std::vector<std::unordered_map<std::string, double>> offsets = {
            {{"x", spacing}, {"y", spacing}},
            {{"x", spacing}, {"y", -spacing}},
            {{"x", -spacing}, {"y", -spacing}},
            {{"x", -spacing}, {"y", spacing}},
        };

        for (size_t idx = 0; idx < module_prefixes_.size(); ++idx)
        {
            const auto &prefix = module_prefixes_[idx];
            modules_helpers_[prefix] = std::make_unique<ModuleKinematics>(prefix, 0.4, offsets[idx]);
        }

        // Log that I'm complete
        start_up_print();

    }

private:
    void start_up_print()
    {
        const std::string log_prefix = "\033[1;32m[HermesRobotNode]\033[0m";
        const std::string log_modules_color = "\033[1;34m";
        const std::string log_reset_color = "\033[0m";

        RCLCPP_INFO(this->get_logger(),
                    "%s Ready with modules: %s%s%s",
                    log_prefix.c_str(), log_modules_color.c_str(), module_prefix_string_.c_str(), log_reset_color.c_str());
    }

    // Command velocity callback
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // print out the fatal flag
        if (fatal_flag_)
            return;
        commanded_vel_ = *msg;
        // RCLCPP_INFO(this->get_logger(), "Received cmd_vel: vx=%.2f vy=%.2f wz=%.2f",
        //             msg->linear.x, msg->linear.y, msg->angular.z);
    }

    void split_module_prefixes(const std::string &prefix_string)
    {
        std::stringstream ss(prefix_string);
        std::string token;
        while (std::getline(ss, token, ','))
        {
            module_prefixes_.push_back(token);
        }
    }

    void update_odometry() // this is the wheel_based_update_odomotry
    {

        // -------- Error handling --------
        double total_error = 0.0;
        if (!module_errors_.empty())
        {
            for (const auto &kv : module_errors_)
                total_error += std::abs(kv.second);
            total_error /= module_errors_.size();
        }

        // print the total error
        // RCLCPP_INFO(this->get_logger(), "Total error: %.2f", total_error);


        // Compute the wheel speed and angle for each module
        for (size_t idx = 0; idx < module_prefixes_.size(); ++idx)
        {
            const auto &helper = modules_helpers_[module_prefixes_[idx]];

            auto [wheel_angle, computed_wheel_speed] = helper->compute(
                {commanded_vel_.linear.x, commanded_vel_.linear.y, commanded_vel_.angular.z},
                commanded_vel_.angular.z,
                robot_positions_["angular_z"]
            );

            auto wheel_speed = computed_wheel_speed;

            if (total_error > 0.1)
            {
                wheel_speed = 0.0;
            }

            // Send the command to the module
            geometry_msgs::msg::Twist twist_msg;
            twist_msg.linear.x = wheel_speed;  // Set the linear.x to the computed wheel speed
            twist_msg.angular.z = wheel_angle;
            drive_publishers_[idx]->publish(twist_msg);
        }

        // Use the commanded velocity to update the odometry (this will be replaced with real kinematics)
        if (total_error > 0.1)
        {
            RCLCPP_WARN(this->get_logger(), "\033[1;31mTotal Pivot error too high for robot (%.2f), halting drive until error is below threshold.\033[0m", total_error);
        }
        else
        {
            // when the total error is low, update the robot position
            double dt = 1.0 / odom_frequency_;
            robot_positions_["x_pos"] += commanded_vel_.linear.x * dt;
            robot_positions_["y_pos"] += commanded_vel_.linear.y * dt;
        }
        // robot_positions_["angular_z"] += commanded_vel_.angular.z * dt;


        // Update odometry here (basic stub)
        auto now = this->get_clock()->now();
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        odom.pose.pose.position.x = robot_positions_["y_pos"] * 1.2;
        odom.pose.pose.position.y = -robot_positions_["x_pos"]* 1.2;
        odom.pose.pose.orientation.w = std::cos(robot_positions_["angular_z"] / 2.0);
        odom.pose.pose.orientation.z = std::sin(robot_positions_["angular_z"] / 2.0);
        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_footprint";
        t.transform.translation.x = robot_positions_["y_pos"]* 1.2;
        t.transform.translation.y = -robot_positions_["x_pos"]* 1.2;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = std::sin(robot_positions_["angular_z"] / 2.0);
        t.transform.rotation.w = std::cos(robot_positions_["angular_z"] / 2.0);
        tf_broadcaster_->sendTransform(t);
    }

    std::string module_prefix_string_;
    std::vector<std::string> module_prefixes_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> error_subs_;
    std::unordered_map<std::string, double> module_errors_;


    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> wheel_position_subs_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> pivot_angle_subs_;

    std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> drive_publishers_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr robot_angle_pub_;


    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr odom_timer_;


    // Pivot positions as Float64 dict
    std::unordered_map<std::string, double> robot_positions_;
    std::unordered_map<std::string, double> wheel_positions_;
    std::unordered_map<std::string, double> pivot_angles_;
    std::unordered_map<std::string, std::unique_ptr<ModuleKinematics>> modules_helpers_;

    geometry_msgs::msg::Twist commanded_vel_;
    sensor_msgs::msg::Imu imu_data_;
    double odom_frequency_;
    bool fatal_flag_;
    rclcpp::TimerBase::SharedPtr robot_angle_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HermesRobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}