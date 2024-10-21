#ifndef FIELDSTRUCT
#define FIELDSTRUCT

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h> 
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

/**
 * @class Crop
 * @brief A class for defining the characteristics of individual crops within the crop guard simulation
 */
class Crop {
public:
    double moisture_;
    double nitrate_;
    geometry_msgs::msg::Point odom_;
};

class Settings {
public:
    unsigned int i_;
    unsigned int n_;
    bool rand_;
    unsigned int layout_;
    unsigned int wacky_crops_;
};