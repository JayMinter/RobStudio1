#ifndef CYLINDER_HPP
#define CYLINDER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <vector>
#include <utility>
#include <tf2/utils.h> 
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

/**
 * @class Cylinder
 * @brief A class for detecting cylindrical objects from laser scan data and visualizing them in RViz.
 */
class Cylinder : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the Cylinder node.
     */
    Cylinder();

private:
    /**
     * @brief Callback function to process the LaserScan data.
     * 
     * @param scan The incoming laser scan data.
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    /**
     * @brief Callback function to process the Odometry data.
     * 
     * @param odom The incoming odometry data.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);

    /**
     * @brief Function to detect and count segments in the laser scan data.
     * 
     * @param scan The laser scan data.
     * @return A vector of segments, each represented as a vector of geometry_msgs::msg::Point.
     */
    std::vector<std::vector<geometry_msgs::msg::Point>> countSegments(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    /**
     * @brief Function to detect if a segment forms a cylinder.
     * 
     * @param segment A segment of laser scan points.
     */
    void detectCylinder(const std::vector<geometry_msgs::msg::Point> &segment);

    /**
     * @brief Function to find the center of a detected cylinder.
     * 
     * @param P1 The first point of the cylinder segment.
     * @param P2 The last point of the cylinder segment.
     * @param r The radius of the cylinder.
     * @return The center point of the cylinder.
     */
    geometry_msgs::msg::Point findCentre(geometry_msgs::msg::Point P1, geometry_msgs::msg::Point P2, double r);

    /**
     * @brief Checks if a detected center is near any previously detected centers.
     * 
     * @param centre The newly detected center.
     * @return True if the center is near an existing center, false otherwise.
     */
    bool checkExisting(geometry_msgs::msg::Point centre);

    /**
     * @brief Function to visualize a segment as a line strip in RViz.
     * 
     * @param segment A segment of laser scan points.
     */
    void visualizeSegment(const std::vector<geometry_msgs::msg::Point> &segment);

    /**
     * @brief Function to create a marker representing a detected cylinder.
     * 
     * @param pt The center point of the cylinder.
     * @return A visualization_msgs::msg::Marker representing the cylinder.
     */
    visualization_msgs::msg::Marker produceMarkerCylinder(geometry_msgs::msg::Point pt);

    /**
     * @brief Converts a local point (from laser scan) to global coordinates using the current odometry data.
     * 
     * @param global The current global odometry information.
     * @param local The local point from laser scan data.
     * @return The point in global coordinates.
     */
    geometry_msgs::msg::Point localToGlobal(const nav_msgs::msg::Odometry &global, const geometry_msgs::msg::Point &local);

    /**
     * @brief Checks if a segment forms a wall.
     * 
     * @param segmentVector The segment points.
     * @return True if the segment forms a wall, false otherwise.
     */
    bool isThisAWall(const std::vector<geometry_msgs::msg::Point> &segmentVector);

    /**
     * @brief Checks if a segment forms a corner.
     * 
     * @param segmentVector The segment points.
     * @return True if the segment forms a corner, false otherwise.
     */
    bool isThisACorner(const std::vector<geometry_msgs::msg::Point> &segmentVector);

    bool isOnKnownObject(const geometry_msgs::msg::Point &point);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; ///< Subscription to laser scan data.
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; ///< Subscription to odometry data.
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_; ///< Publisher for RViz markers.
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    nav_msgs::msg::Odometry currentOdom; ///< Stores the current odometry data.
    int ct_ = 0; ///< Marker ID counter for unique markers.
    std::vector<geometry_msgs::msg::Point> centres; ///< Vector storing detected cylinder centers.
    bool firstCent = true; ///< Flag to check if it's the first detected cylinder.
    nav_msgs::msg::OccupancyGrid map_data_;  

};

#endif // CYLINDER_HPP
