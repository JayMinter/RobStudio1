// #ifndef CYLINDER_HPP_
// #define CYLINDER_HPP_

// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <nav_msgs/msg/occupancy_grid.hpp>
// #include <visualization_msgs/msg/marker.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include "std_msgs/msg/bool.hpp"
// #include <tf2/utils.h> 
// #include <cmath>
// #include <vector>

// #include <Eigen/Dense>  // Eigen library for linear algebra (used in least squares fitting)
// #include <iostream>

// #include <mutex>

// class Cylinder : public rclcpp::Node
// {
// public:
//     Cylinder();
//     ~Cylinder();
    
//     bool detectCylinder(const sensor_msgs::msg::LaserScan::SharedPtr& scan_data);

//     void processlatestScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan_data);

// private:
//     // ROS 2 subscriptions and publishers
//     // rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
//     rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
//     rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
//     rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detection_publisher_;

//     rclcpp::TimerBase::SharedPtr command_timer_;

//     // Callbacks for the subscriptions
//     void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
//     void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    

//     // Cylinder detection methods
//     void countSegments(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    

//     // Helper methods for cylinder detection and checking
//     geometry_msgs::msg::Point findCentre(geometry_msgs::msg::Point P1, geometry_msgs::msg::Point P2, double r);
//     bool checkExisting(geometry_msgs::msg::Point centre);
//     geometry_msgs::msg::Point localToGlobal(const nav_msgs::msg::Odometry &global, const geometry_msgs::msg::Point &local);


    

//     // Produces a marker for visualization in RViz
//     visualization_msgs::msg::Marker produceMarkerCylinder(geometry_msgs::msg::Point pt);

//     std::vector<float> smoothScanData(const sensor_msgs::msg::LaserScan scan, int window_size);

//     void publishCircleMotion(geometry_msgs::msg::Point cylinder);

//     double computeSteering(nav_msgs::msg::Odometry origin, geometry_msgs::msg::Point goal);
//     void stopTurtle();

//     nav_msgs::msg::Odometry getOdometry(void);

//     void publishCommands(double x, double z);


//     double computePerpendicularAngle(const nav_msgs::msg::Odometry &odom, geometry_msgs::msg::Point goal);

//     // ROS data storage
//     nav_msgs::msg::Odometry currentOdom_;


//     // Vector to store centers of detected cylinders
//     std::vector<geometry_msgs::msg::Point> centres;

//     std::vector<std::vector<geometry_msgs::msg::Point>> segmentVector_;
//     std::mutex segmentMutex_;

//     // Flags and counters
//     bool firstCent = true;
//     int ct_ = 0;
    
//      std::unique_ptr<std::thread> thread_;
//      bool running_;

//      sensor_msgs::msg::LaserScan latest_scan_;
//      std::mutex scan_mutex_;
    
// };

// #endif  // CYLINDER_HPP_


#ifndef CYLINDER_HPP_
#define CYLINDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <mutex>
#include <thread>

class Cylinder : public rclcpp::Node
{
public:
    Cylinder();
    ~Cylinder();

    bool detectCylinder(const sensor_msgs::msg::LaserScan::SharedPtr& scan_data);
    void processlatestScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan_data);
    void stopTurtle();
    geometry_msgs::msg::Point getCylinder();
    void publishCircleMotion(geometry_msgs::msg::Point cylinder);

    geometry_msgs::msg::Point getSegmentMidpoints(const sensor_msgs::msg::LaserScan::SharedPtr &scan);
    nav_msgs::msg::Odometry getOdometry();
    bool odomreceived();

private:
    // ROS 2 subscriptions and publishers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detection_publisher_;

    // ROS 2 Timer (if needed, otherwise remove this declaration)
    rclcpp::TimerBase::SharedPtr command_timer_;

    // Callback functions
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

    // Cylinder detection methods
    void countSegments(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // Helper methods
    geometry_msgs::msg::Point findCentre(geometry_msgs::msg::Point P1, geometry_msgs::msg::Point P2, double r);
    bool checkExisting(geometry_msgs::msg::Point centre);
    geometry_msgs::msg::Point localToGlobal(const nav_msgs::msg::Odometry &global, const geometry_msgs::msg::Point &local);
    visualization_msgs::msg::Marker produceMarkerCylinder(geometry_msgs::msg::Point pt);
    std::vector<float> smoothScanData(const sensor_msgs::msg::LaserScan scan, int window_size);
    
    double computeSteering(nav_msgs::msg::Odometry origin, geometry_msgs::msg::Point goal);
    
    
    void publishCommands(double x, double z);
    double computePerpendicularAngle(const nav_msgs::msg::Odometry &odom, geometry_msgs::msg::Point goal);

    // Data storage for ROS messages and detected cylinder centers
    nav_msgs::msg::Odometry currentOdom_;
    std::vector<geometry_msgs::msg::Point> centres;
    std::vector<std::vector<geometry_msgs::msg::Point>> segmentVector_;
    std::mutex segmentMutex_;

    // Flags and counters
    bool firstCent = true;
    int ct_ = 0;

    // Threading for separate tasks
    std::unique_ptr<std::thread> thread_;
    bool running_;

    // Latest scan data and associated mutex
    sensor_msgs::msg::LaserScan latest_scan_;
    std::mutex scan_mutex_;

    geometry_msgs::msg::Point cylinder_centre_;

    bool odometry_received_;

    
};

#endif  // CYLINDER_HPP_
