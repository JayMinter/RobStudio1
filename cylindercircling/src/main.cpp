#include "main.hpp"

Main::Main(std::shared_ptr<Cylinder> cylinder)
    : Node("main_node"),
      goal_reached_(false),
      interrupted_(false),
      cylinderPtr_(cylinder),
      iscylindergoal_(false)
{
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Subscriber to detect cylinder presence
    detection_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/cylinder_detected", 10,
        std::bind(&Main::detection_callback, this, std::placeholders::_1));

    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Main::scanCallback, this, std::placeholders::_1));

    goal_subscriber_ = this->create_subscription<nav2_msgs::action::NavigateToPose::Goal>( // geometry_msgs::msg::PoseStamped
        "/navigate_to_pose/_action/goal", 10,
        std::bind(&Main::goalCallback, this, std::placeholders::_1));

    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


}

Main::~Main()
{
}

// void Main::run()
// {
//     loadDemoGoals();
//     publishGoals();
// }

void Main::run()
{
    if (waitForOdometryAndSetInitialPose())  // Proceed only if odometry is available and initial pose is set
    {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        loadDemoGoals();
        publishGoals();
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot proceed with goals as initial pose estimate was not set.");
    }
}

// bool Main::waitForOdometryAndSetInitialPose()
// {
//     RCLCPP_INFO(this->get_logger(), "Waiting for odometry data...");

//     // Wait until odometry data is received
//     while (rclcpp::ok() && !cylinderPtr_->odomreceived())
//     {
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }

//     if (cylinderPtr_->odomreceived())
//     {
//         RCLCPP_INFO(this->get_logger(), "Odometry data received. Setting initial pose estimate.");

//         // Set the initial pose estimate based on the odometry data
//         geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
//         initial_pose.header.stamp = this->now();
//         initial_pose.header.frame_id = "map";

//         initial_pose.pose.pose.position.x = cylinderPtr_->getOdometry().pose.pose.position.x;
//         initial_pose.pose.pose.position.y = cylinderPtr_->getOdometry().pose.pose.position.y;
//         initial_pose.pose.pose.position.z = 0.0;

//         initial_pose.pose.pose.orientation = cylinderPtr_->getOdometry().pose.pose.orientation;

//         // Publish the initial pose estimate
//         initial_pose_pub_->publish(initial_pose);
//         RCLCPP_INFO(this->get_logger(), "Initial pose estimate set.");
//         std::this_thread::sleep_for(std::chrono::seconds(5));
//         return true;  // Successfully set the initial pose
//     }

//     RCLCPP_WARN(this->get_logger(), "Failed to receive odometry data.");
//     return false;  // Failed to set initial pose due to missing odometry data
// }

bool Main::waitForOdometryAndSetInitialPose()
{
    RCLCPP_INFO(this->get_logger(), "Waiting for odometry data...");

    // Wait until odometry data is received
    while (rclcpp::ok() && !cylinderPtr_->odomreceived())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (cylinderPtr_->odomreceived())
    {
        RCLCPP_INFO(this->get_logger(), "Odometry data received. Setting initial pose estimate.");

        // Set the initial pose estimate based on the odometry data
        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
        initial_pose.header.stamp = this->now();
        initial_pose.header.frame_id = "map";

        initial_pose.pose.pose.position.x = cylinderPtr_->getOdometry().pose.pose.position.x;
        initial_pose.pose.pose.position.y = cylinderPtr_->getOdometry().pose.pose.position.y;
        initial_pose.pose.pose.position.z = 0.0;

        initial_pose.pose.pose.orientation = cylinderPtr_->getOdometry().pose.pose.orientation;

        // Publish the initial pose estimate
        initial_pose_pub_->publish(initial_pose);
        RCLCPP_INFO(this->get_logger(), "Initial pose estimate published.");

        // Wait for localization to confirm
        int attempts = 0;
        bool localized = false;

        while (rclcpp::ok() && !localized && attempts < 50) // Adjust attempts as necessary
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            attempts++;

            // Here, we should check for a TF transform from "map" to "odom" to confirm localization
            try
            {
                // Assuming you have a tf buffer and listener set up
                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
                localized = true; // Transform available, so we assume localization succeeded
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_INFO(this->get_logger(), "Waiting for localization... (%d)", attempts);
            }
        }

        if (localized)
        {
            RCLCPP_INFO(this->get_logger(), "Localization confirmed. Proceeding to goal sequence.");
            return true;  // Successfully localized and set initial pose
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Localization failed after setting initial pose.");
        }
    }

    RCLCPP_WARN(this->get_logger(), "Failed to receive odometry data.");
    return false;  // Failed to set initial pose due to missing odometry data or localization failure
}



void Main::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    std::lock_guard<std::mutex> lock(scan_mutex1_); // Protect access to the scan data
    latest_scan2_ = scan;

    // // Detect cylinder using the latest scan data
    // bool detected = cylinderPtr_->detectCylinder(latest_scan2_);
    // // RCLCPP_INFO(this->get_logger(), "Inside Scan Callback");
    // if (detected && !goal_reached_) // Only send the goal if not already reached
    // {
    //     RCLCPP_INFO(this->get_logger(), "Cylinder detected. Sending goal towards cylinder.");

    //     // Get the cylinder’s position and create a goal
    //     geometry_msgs::msg::Point cylinder_center = cylinderPtr_->getCylinder();

    //     geometry_msgs::msg::PoseStamped cylinder_goal = createGoalFromCylinderCenter(cylinder_center);

    //     // Send the goal towards the cylinder
    //     sendGoal(cylinder_goal);
    //     goal_reached_ = false; // Reset goal status
    // }

    // bool completedCircularPath = cylinderPtr_->getCompletedCircle();

    // if (completedCircularPath)
    // {
    //     sendGoal(initial_goal_);
    // }
}

geometry_msgs::msg::PoseStamped Main::getInitialGoal()
{
    return initial_goal_;
}

void Main::goalCallback(const nav2_msgs::action::NavigateToPose::Goal msg)
{
    // Record the initial goal only if it hasn't been set
    if (!initial_goal_set_)
    {
        initial_goal_ = msg.pose;
        initial_goal_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Initial goal recorded.");
    }
}

void Main::sendGoal(const geometry_msgs::msg::PoseStamped &goal)
{
    if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal;

    RCLCPP_INFO(this->get_logger(), "Sending goal at x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>> handle)
    {
        if (!handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by the server");
            this->goal_handle_ = handle; // Store the current goal handle
        }
    };
    send_goal_options.feedback_callback = std::bind(&Main::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&Main::resultCallback, this, std::placeholders::_1);

    action_client_->async_send_goal(goal_msg, send_goal_options);
}

void Main::feedbackCallback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
    // RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f", feedback->distance_remaining);
}

void Main::loadDemoGoals()
{
    // Define the coordinates and assign orientations based on the goal index
    std::vector<std::pair<double, double>> goals = {
        {-0.35, -4}, {2.1, -3.97}, {4.5, -3.84}, {4.3, -1.5}, {2.1, -1.42}, {-0.45, -1.37}, {-0.37, 1.69}, {2.10, 1.57}, {4.33, 1.46}, {4.44, 4.14}, {2, 4.26}, {-0.38, 4.25}, {-4, 0}};

    // Load goals with specific orientations
    for (size_t i = 0; i < goals.size(); ++i)
    {
        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = this->now();
        goal.pose.position.x = goals.at(i).first;
        goal.pose.position.y = goals.at(i).second;
        goal.pose.position.z = 0.0;
        goal.header.frame_id = "map";

        if ((i >= 0 && i <= 2) || (i >= 6 && i <= 8))
        {

            goal.pose.orientation.w = 1.0;
            goal.pose.orientation.z = 0.0;
        }
        else
        {

            goal.pose.orientation.w = 0.0;
            goal.pose.orientation.z = 1.0;
        }

        demo_goals_.push_back(goal);
    }
}
void Main::publishGoals()
{
    for (size_t i = 0; i < demo_goals_.size(); ++i)
    {
        auto goal = demo_goals_[i];
        goal_reached_ = false;
        iscylindergoal_ = false;  // Ensure this flag is reset at the start of each main goal
        sendGoal(goal);

        // Main goal loop to wait for goal completion or interruption
        while (!goal_reached_ && rclcpp::ok())
        {
            rclcpp::sleep_for(std::chrono::milliseconds(100)); // Small loop delay
            bool detection = cylinderPtr_->detectCylinder(latest_scan2_);

            // Handle cylinder detection only if the robot is not currently processing a cylinder goal
            if (detection && !goal_reached_ && !iscylindergoal_)
            {
                RCLCPP_INFO(this->get_logger(), "Cylinder detected. Interrupting navigation to circle cylinder.");

                // Set the flag to indicate a temporary cylinder goal
                iscylindergoal_ = true;

                // Get the cylinder’s position and create a goal to approach it
                geometry_msgs::msg::Point cylinder_center = cylinderPtr_->getCylinder();
                geometry_msgs::msg::PoseStamped cylinder_goal = createGoalFromCylinderCenter(cylinder_center);
                
                // Send the goal towards the cylinder
                sendGoal(cylinder_goal);
                goal_reached_ = false;  // Reset goal status for the cylinder goal

                // Wait until the cylinder goal is reached
                while (!goal_reached_ && rclcpp::ok())
                {
                    rclcpp::sleep_for(std::chrono::milliseconds(100));
                }

                // Start circling the cylinder if the cylinder goal was reached
                if (goal_reached_)
                {
                    RCLCPP_INFO(this->get_logger(), "Starting circular motion around detected cylinder.");
                    cylinderPtr_->publishCircleMotion(cylinder_center);
                }

                // Reset cylinder-related flags and continue to the original main goal
                iscylindergoal_ = false;  // Reset to indicate we're back to the main goal
                goal_reached_ = false;    // Reset goal completion for the main goal
                RCLCPP_INFO(this->get_logger(), "Resuming navigation to main goal %ld.", i + 1);
                sendGoal(goal);  // Resend the original main goal after handling the cylinder
            }
        }

        // If the goal was reached without interruptions, log the completion
        if (goal_reached_)
        {
            RCLCPP_INFO(this->get_logger(), "Main goal %ld reached successfully.", i + 1);
        }

        // Optional delay to ensure the robot is stable before moving to the next goal
        rclcpp::sleep_for(std::chrono::seconds(3));
    }

    RCLCPP_INFO(this->get_logger(), "All goals have been reached.");
}


// Result callback for navigation goal
void Main::resultCallback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result)
{
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal reached successfully.");
        goal_reached_ = true;
        if(iscylindergoal_){
        cylinderPtr_->sendMessage();
        }
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted.");
        goal_reached_ = true;
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled.");
        goal_reached_ = true;
        break;
    default:
        RCLCPP_WARN(this->get_logger(), "Unknown result code.");
        break;
    }
}

void Main::detection_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    interrupted_ = msg->data;
    if (interrupted_ && goal_reached_) // Ensure both conditions are met
    {
        RCLCPP_INFO(this->get_logger(), "Cylinder goal reached. Starting circular motion.");

        // Get the cylinder’s position and start the circling motion
        geometry_msgs::msg::Point cylinder_center2 = cylinderPtr_->getCylinder();
        cylinderPtr_->publishCircleMotion(cylinder_center2);

        // Reset the goal_reached_ flag after circling to prevent retriggering
        goal_reached_ = false;
    }
    else if (!interrupted_)
    {
        RCLCPP_INFO(this->get_logger(), "Cylinder no longer detected. Resuming initial goal.");

        // Retrieve the initial goal and send the robot back to it
        geometry_msgs::msg::PoseStamped initial_goal = getInitialGoal();
        sendGoal(initial_goal);
    }
}

geometry_msgs::msg::PoseStamped Main::createGoalFromCylinderCenter(const geometry_msgs::msg::Point &cylinder_center)
{
    geometry_msgs::msg::PoseStamped cylinder_goal;
    cylinder_goal.header.frame_id = "map";
    cylinder_goal.header.stamp = rclcpp::Clock().now();

    cylinder_goal.pose.position.x = cylinder_center.x - 0.7;
    cylinder_goal.pose.position.y = cylinder_center.y;
    cylinder_goal.pose.position.z = 0.0;

    cylinder_goal.pose.orientation.w = 1.0; // Neutral orientation

    return cylinder_goal;
}

int main(int argc, char *argv[])
{
    try
    {
        rclcpp::init(argc, argv);

        auto cylinder_node = std::make_shared<Cylinder>();
        auto main_node = std::make_shared<Main>(cylinder_node);

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(cylinder_node);
        executor.add_node(main_node);

        // Run `main_node->run()` in a separate thread so it doesn't block executor spin
        std::thread main_thread([&main_node]()
                                { main_node->run(); });

        executor.spin();

        main_thread.join(); // Ensure the thread completes before shutting down
        rclcpp::shutdown();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception caught in main: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

