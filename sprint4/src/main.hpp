#ifndef MAIN_HPP
#define MAIN_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include <vector>
#include <cmath>

#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


#include "cylinder.hpp"


using NavigateToPose = nav2_msgs::action::NavigateToPose;

class Main : public rclcpp::Node
{
public:
     Main(std::shared_ptr<Cylinder> cylinder);
    ~Main();

    void publishGoals();
    void run();
    void loadDemoGoals();

    void stopNavigationAndRobot();      
    void forceStopRobot();    

private:

bool waitForOdometryAndSetInitialPose();
// rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
//     std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> detection_subscriber_;
//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
//     std::vector<geometry_msgs::msg::PoseStamped> demo_goals_;
//     bool goal_reached_;
//     bool performed_scan_;
//     bool interrupted_;

//     std::mutex scan_mutex1_;
//     sensor_msgs::msg::LaserScan::SharedPtr latest_scan2_;

//     // Shared pointer to Cylinder node
//     std::shared_ptr<Cylinder> cylinderPtr_;

//     // Method to send a goal using the action client
//     void sendGoal(const geometry_msgs::msg::PoseStamped &goal);

//     void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

//     // Callbacks for the action client
//     void goalResponseCallback(rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle);
//     void feedbackCallback(rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
//                           const std::shared_ptr<const NavigateToPose::Feedback> feedback);
//     void resultCallback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result);

//     // Callback for cylinder detection
//     void detection_callback(const std_msgs::msg::Bool::SharedPtr msg);

//     // Helper methods
    
//     void scanner();
//     geometry_msgs::msg::PoseStamped createRotationGoal(const geometry_msgs::msg::PoseStamped& base_goal, double z, double w);
// };




    // Action client for navigation goals
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_; // Store goal handle for specific goal cancellation

    // Publisher for velocity commands to stop the robot
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // Callback functions
    void sendGoal(const geometry_msgs::msg::PoseStamped &goal);
    void goalResponseCallback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle);
    void feedbackCallback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
                          const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
    void resultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result);
    void detection_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    geometry_msgs::msg::PoseStamped createRotationGoal(const geometry_msgs::msg::PoseStamped &base_goal, double z, double w);

    visualization_msgs::msg::Marker produceMarkerPlant(geometry_msgs::msg::Point pt);

    // Other member variables as needed
    bool goal_reached_;
    bool interrupted_;
    std::vector<geometry_msgs::msg::PoseStamped> demo_goals_;

    std::shared_ptr<Cylinder> cylinderPtr_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan2_;
    std::mutex scan_mutex1_;

    // void loadDemoGoals();
    void scanner();

    geometry_msgs::msg::PoseStamped createGoalFromCylinderCenter(const geometry_msgs::msg::Point& cylinder_center);

    void imagecallback(const sensor_msgs::msg::Image::SharedPtr msg);

    bool performed_scan_;  // Tracks if a scan has been performed
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detection_subscriber_;  // Subscription for cylinder detection
rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;  // Subscription for scan data

rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;


    cv::Mat latest_frame_;

     int ct_ = 0;

     rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
         std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;



};

#endif // MAIN_HPP