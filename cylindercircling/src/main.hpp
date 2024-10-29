#ifndef MAIN_HPP
#define MAIN_HPP

#include <memory>
#include <thread>
#include <mutex>
#include <vector>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "cylinder.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>


class Main : public rclcpp::Node
{
public:
    explicit Main(std::shared_ptr<Cylinder> cylinder);
    ~Main();

    void sendGoal(const geometry_msgs::msg::PoseStamped &goal);
    void run();
    
private:

geometry_msgs::msg::PoseStamped getInitialGoal();

void loadDemoGoals();
void publishGoals();

bool waitForOdometryAndSetInitialPose();




    void goalCallback(const nav2_msgs::action::NavigateToPose::Goal msg);
    void detection_callback(const std_msgs::msg::Bool::SharedPtr msg);

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    
    // Missing callback functions for action feedback and result
    void feedbackCallback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
                          const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);

    void resultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result);

    void stopNavigationAndRobot();
    void forceStopRobot();
    geometry_msgs::msg::PoseStamped createGoalFromCylinderCenter(const geometry_msgs::msg::Point &cylinder_center);

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detection_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

    std::shared_ptr<Cylinder> cylinderPtr_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan2_;
    bool goal_reached_;
    bool interrupted_;

    bool iscylindergoal_;
    std::mutex scan_mutex1_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;


        geometry_msgs::msg::PoseStamped initial_goal_;
    bool initial_goal_set_;
    rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Goal>::SharedPtr goal_subscriber_;

    std::vector<geometry_msgs::msg::PoseStamped> demo_goals_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

};

#endif // MAIN_HPP
