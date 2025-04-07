#pragma once

#include <iostream>
#include <cmath>
#include <string>
#include <unordered_map>
#include <tuple>

class ModuleKinematics {
public:
  ModuleKinematics(const std::string& name, double wheel_size, const std::unordered_map<std::string, double>& module_position, double starting_angle = 0.0)
    : name_(name), wheel_size_(wheel_size), module_position_(module_position), angle_(starting_angle) {}

  static double normalize_angle(double angle) {
    return std::fmod(std::fmod(angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
  }

  static double shortest_angle(double current_angle, double target_angle) {
    // print the input and output
    double sub1 = target_angle - current_angle + M_PI;
    double sub2 = std::fmod(sub1, 2 * M_PI);
    double diff = sub2 - M_PI;
    return current_angle + diff;
  }

  static double compute_wheel_speed(double Vx, double Vy, double omega, double Xi, double Yi) {
    return std::sqrt(std::pow(Vx - omega * Yi, 2) + std::pow(Vy + omega * Xi, 2));
  }

  static double compute_wheel_angle(double Vx, double Vy, double omega, double Xi, double Yi) {
    return std::atan2(Vy + omega * Xi, Vx - omega * Yi);
  }

  std::pair<double, double> compute(
    const std::tuple<double, double, double>& robotVelocity,
    double robot_angular_velocity,
    double robotAngle
  ) {
    double x = module_position_.at("x");
    double y = module_position_.at("y");

    double rotatedX = x * std::cos(robotAngle) - y * std::sin(robotAngle);
    double rotatedY = x * std::sin(robotAngle) + y * std::cos(robotAngle);

    double vx = std::get<0>(robotVelocity);
    double vy = std::get<1>(robotVelocity);

    double wheel_speed = compute_wheel_speed(vx, vy, robot_angular_velocity, rotatedX, rotatedY) / wheel_size_;
    double target_angle = compute_wheel_angle(vx, vy, robot_angular_velocity, rotatedX, rotatedY) - robotAngle;
    double wheel_angle = shortest_angle(angle_, target_angle);
    angle_ = wheel_angle;

    return { std::round(wheel_angle * 100.0) / 100.0, std::round(wheel_speed * 100.0) / 100.0 };
  }

private:
  std::string name_;
  double wheel_size_;
  std::unordered_map<std::string, double> module_position_;
  double angle_;
};
