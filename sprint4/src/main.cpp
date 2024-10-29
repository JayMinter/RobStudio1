#include "main.hpp"

Main::Main(std::shared_ptr<Cylinder> cylinder)
    : Node("main_node"),
      goal_reached_(false),
      performed_scan_(true),
      interrupted_(false),
      cylinderPtr_(cylinder)
{
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    rclcpp::QoS publisher_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    publisher_qos.keep_last(10).reliable();

    detection_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/cylinder_detected", 10,
        std::bind(&Main::detection_callback, this, std::placeholders::_1));

    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Main::scanCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<std_msgs::msg::Bool>("is_green", 10);

    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&Main::imagecallback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/visualization_marker", publisher_qos);

           initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

Main::~Main()
{
}

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
    // cylinderPtr_->processlatestScan(latest_scan2_);
    // RCLCPP_INFO(this->get_logger(), "Scan data updated.");
}

// void Main::sendGoal(const geometry_msgs::msg::PoseStamped &goal)
// {
//     if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
//     {
//         RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
//         return;
//     }

//     auto goal_msg = NavigateToPose::Goal();
//     goal_msg.pose = goal;

//     RCLCPP_INFO(this->get_logger(), "Sending goal at x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);

//     auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
//     send_goal_options.goal_response_callback = std::bind(&Main::goalResponseCallback, this, std::placeholders::_1);
//     send_goal_options.feedback_callback = std::bind(&Main::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
//     send_goal_options.result_callback = std::bind(&Main::resultCallback, this, std::placeholders::_1);

//     action_client_->async_send_goal(goal_msg, send_goal_options);
// }

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

void Main::goalResponseCallback(rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle)
{
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by the server, waiting for result");
    }
}

void Main::feedbackCallback(rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
                            const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    double distance_remaining = feedback->distance_remaining;
    // RCLCPP_INFO(this->get_logger(), "Distance remaining to goal: %.2f", distance_remaining);
}

void Main::resultCallback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result)
{
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal reached successfully.");
        goal_reached_ = true;
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
    if (interrupted_)
    {
        RCLCPP_INFO(this->get_logger(), "Cylinder detected, pausing navigation...");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Cylinder cleared, resuming navigation...");
    }
}

void Main::publishGoals()
{

    for (size_t i = 0; i < demo_goals_.size(); ++i)
    {
        auto goal = demo_goals_[i];
        goal_reached_ = false;
        sendGoal(goal);

        // Main goal loop to wait for goal completion or interruption
        while (!goal_reached_ && rclcpp::ok())
        {
            rclcpp::sleep_for(std::chrono::milliseconds(100)); // Small loop delay
            bool detection = cylinderPtr_->detectCylinder(latest_scan2_);
        }
            // Check for cylinder detection and interruption
        //     if (detection)
        //     {

        //         // stopNavigationAndRobot();
        //         geometry_msgs::msg::Point cylinder_center = cylinderPtr_->getCylinder();
        //         geometry_msgs::msg::PoseStamped cylinder_goal = createGoalFromCylinderCenter(cylinder_center);

        //         // Send the goal to navigate to the offset cylinder location
        //          goal_reached_ = false;
        //         sendGoal(cylinder_goal);
        //         while (!goal_reached_ && rclcpp::ok())
        //         {
        //             rclcpp::sleep_for(std::chrono::milliseconds(100));
        //         }

        //         // Call publishCircleMotion once the cylinder goal is reached
        //         if (goal_reached_ && rclcpp::ok())
        //         {
        //             cylinderPtr_->publishCircleMotion(cylinder_center);
        //             RCLCPP_INFO(this->get_logger(), "Cylinder goal reached. Starting circular motion around the cylinder.");
        //         }
        //         RCLCPP_INFO(this->get_logger(), "Goal interrupted. Canceling current goal...");
        //         // action_client_->async_cancel_all_goals();
        //         // cylinderPtr_->stopTurtle();

        //         // Wait for the interruption to be resolved
        //         while (interrupted_ && rclcpp::ok())
        //         {
        //             rclcpp::sleep_for(std::chrono::milliseconds(100));
        //         }

        //         // Resend the goal after interruption is cleared
        //         if (!interrupted_ && rclcpp::ok())
        //         {
        //             RCLCPP_INFO(this->get_logger(), "Resending goal after interruption cleared.");
        //             // auto goal = demo_goals_[i];
        //             sendGoal(demo_goals_.at(i));
        //         }
        //     }
        // }

        // Goal reached, reset scan state
        performed_scan_ = false;
        RCLCPP_INFO(this->get_logger(), "Goal %ld reached. Performing rotation.", i + 1);

        // Rotate based on goal index
        geometry_msgs::msg::PoseStamped rotation_goal;
        // if (i >= 0 && i <= 2) // Goals 1-3
        // {
        //     rotation_goal = createRotationGoal(goal, 0.7, 0.7);
        //     goal_reached_ = false;
        //     sendGoal(rotation_goal);
        //     if(goal_reached_){
        //     scanner();
        //     }
        //     rclcpp::sleep_for(std::chrono::seconds(5));
        // }
        // else if (i >= 3 && i <= 8) // Goals 4-9
        // {
        //     rotation_goal = createRotationGoal(goal, 0.7, 0.7);
        //     goal_reached_ = false;
        //     sendGoal(rotation_goal);
        //     if(goal_reached_){
        //     scanner();
        //     }
        //     rclcpp::sleep_for(std::chrono::seconds(5));

        //     rotation_goal = createRotationGoal(goal, -0.7, 0.7);
        //     goal_reached_ = false;
        //     sendGoal(rotation_goal);
        //     if(goal_reached_){
        //     scanner();
        //     }
        // }
        // else if (i >= 9 && i <= 11) // Goals 10-12
        // {
        //     rotation_goal = createRotationGoal(goal, -0.7, 0.7);
        //     goal_reached_ = false;
        //     sendGoal(rotation_goal);
        //     if(goal_reached_){
        //     scanner();
        //     }
        //     rclcpp::sleep_for(std::chrono::seconds(5));
        // }


        if (i >= 0 && i <= 2) // Goals 1-3
        {
            rotation_goal = createRotationGoal(goal, 0.7, 0.7);
            goal_reached_ = false;
            sendGoal(rotation_goal);

            // Wait until the rotation goal is reached before scanning
            while (!goal_reached_ && rclcpp::ok())
            {
                rclcpp::sleep_for(std::chrono::milliseconds(100)); // Small loop delay
            }

            // Perform the scan only after the rotation goal has been achieved
            if (goal_reached_)
            {
                scanner();
            }

            rclcpp::sleep_for(std::chrono::seconds(5));
        }
        else if (i >= 3 && i <= 8) // Goals 4-9
        {
            rotation_goal = createRotationGoal(goal, 0.7, 0.7);
            goal_reached_ = false;
            sendGoal(rotation_goal);

            while (!goal_reached_ && rclcpp::ok())
            {
                rclcpp::sleep_for(std::chrono::milliseconds(100)); // Small loop delay
            }

            if (goal_reached_)
            {
                scanner();
            }

            rclcpp::sleep_for(std::chrono::seconds(5));

            rotation_goal = createRotationGoal(goal, -0.7, 0.7);
            goal_reached_ = false;
            sendGoal(rotation_goal);

            while (!goal_reached_ && rclcpp::ok())
            {
                rclcpp::sleep_for(std::chrono::milliseconds(100)); // Small loop delay
            }

            if (goal_reached_)
            {
                scanner();
            }
        }
        else if (i >= 9 && i <= 11) // Goals 10-12
        {
            rotation_goal = createRotationGoal(goal, -0.7, 0.7);
            goal_reached_ = false;
            sendGoal(rotation_goal);

            while (!goal_reached_ && rclcpp::ok())
            {
                rclcpp::sleep_for(std::chrono::milliseconds(100)); // Small loop delay
            }

            if (goal_reached_)
            {
                scanner();
            }

            rclcpp::sleep_for(std::chrono::seconds(5));
        }

        // Delay before moving to the next goal
        RCLCPP_INFO(this->get_logger(), "Moving to next goal...");
        rclcpp::sleep_for(std::chrono::seconds(5));
    }

    RCLCPP_INFO(this->get_logger(), "All goals have been reached.");
}

geometry_msgs::msg::PoseStamped Main::createGoalFromCylinderCenter(const geometry_msgs::msg::Point &cylinder_center)
{
    geometry_msgs::msg::PoseStamped cylinder_goal;
    cylinder_goal.header.frame_id = "map";
    cylinder_goal.header.stamp = rclcpp::Clock().now();

    // Apply the offset in the x-direction
    cylinder_goal.pose.position.x = cylinder_center.x;
    cylinder_goal.pose.position.y = cylinder_center.y;
    cylinder_goal.pose.position.z = 0.0;

    // Set orientation - neutral/facing forward
    cylinder_goal.pose.orientation.w = 1.0;

    return cylinder_goal;
}

void Main::stopNavigationAndRobot()
{
    // Step 1: Attempt to cancel the specific goal handle if available
    if (goal_handle_)
    {
        auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), cancel_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Specific goal cancellation confirmed.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to cancel the specific goal.");
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "No active goal handle to cancel.");
    }

    // Step 2: Manually override velocity commands with a stop message
    forceStopRobot(); // Override /cmd_vel for a few seconds with zero velocity

    // Step 3: Deactivate the navigation stack nodes if needed
    system("ros2 lifecycle set /controller_server deactivate");
    system("ros2 lifecycle set /bt_navigator deactivate");

    // Step 4: Reset and reinitialize the action client to clear any residual state
    action_client_.reset();
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    RCLCPP_INFO(this->get_logger(), "Navigation and robot successfully stopped.");
}

void Main::forceStopRobot()
{
    auto stop_msg = geometry_msgs::msg::Twist();
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;

    // auto start_time = this->now();
    // auto timeout = rclcpp::Duration::from_seconds(3); // Override for 3 seconds

    // while ((this->now() - start_time) < timeout && rclcpp::ok()) {
    cmd_vel_pub_->publish(stop_msg);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    // }

    RCLCPP_INFO(this->get_logger(), "Force-stopped the robot by overriding cmd_vel.");
}

geometry_msgs::msg::PoseStamped Main::createRotationGoal(const geometry_msgs::msg::PoseStamped &base_goal, double z, double w)
{
    geometry_msgs::msg::PoseStamped rotation_goal = base_goal;
    rotation_goal.pose.orientation.z = z;
    rotation_goal.pose.orientation.w = w;
    return rotation_goal;
}

// void Main::scanner()
// {
//     // Check for image processing to identify the color as green or yellow
//     cv::Mat frame = latest_frame_;

//     // Crop to the center 1/3 of the image
//     int x_start = frame.cols / 3;
//     int x_end = 2 * frame.cols / 3;
//     cv::Mat center_frame = frame(cv::Range::all(), cv::Range(x_start, x_end));

//     // Convert the cropped frame to HSV color space
//     cv::Mat hsv_frame;
//     cv::cvtColor(center_frame, hsv_frame, cv::COLOR_BGR2HSV);

//     // Define HSV ranges for green and yellow
//     cv::Scalar lower_green(35, 50, 50);
//     cv::Scalar upper_green(85, 255, 255);
//     cv::Scalar lower_yellow(20, 100, 100);
//     cv::Scalar upper_yellow(30, 255, 255);

//     // Create masks for green and yellow
//     cv::Mat green_mask, yellow_mask;
//     cv::inRange(hsv_frame, lower_green, upper_green, green_mask);
//     cv::inRange(hsv_frame, lower_yellow, upper_yellow, yellow_mask);

//     // Count non-zero pixels in each mask (representing green and yellow pixels)
//     int green_count = cv::countNonZero(green_mask);
//     int yellow_count = cv::countNonZero(yellow_mask);

//     // Determine predominant color
//     bool is_green = green_count > yellow_count;

//     if (is_green)
//     {
//         RCLCPP_INFO(this->get_logger(), "Current reading: Predominantly Green");
//     }
//     else
//     {
//         RCLCPP_INFO(this->get_logger(), "Current reading: Predominantly Yellow");

//         // Process laser scan data in front segment to find segments and mark the plant location
//         if (latest_scan2_ && !latest_scan2_->ranges.empty())
//         {
//             // Define angle range for the front segment (-15 to +15 degrees)
//             double angle_min_deg_front = -15.0;
//             double angle_max_deg_front = 15.0;

//             // Convert angle range to radians
//             double angle_min_rad_front = angle_min_deg_front * M_PI / 180.0;
//             double angle_max_rad_front = angle_max_deg_front * M_PI / 180.0;

//             // Calculate index range for front subset
//             int start_index_front = static_cast<int>((angle_min_rad_front - latest_scan2_->angle_min) / latest_scan2_->angle_increment);
//             int end_index_front = static_cast<int>((angle_max_rad_front - latest_scan2_->angle_min) / latest_scan2_->angle_increment);

//             // Create a new LaserScan message for the front subset
//             auto subset_scan_front = std::make_shared<sensor_msgs::msg::LaserScan>(*latest_scan2_);
//             subset_scan_front->ranges = std::vector<float>(latest_scan2_->ranges.begin() + start_index_front, latest_scan2_->ranges.begin() + end_index_front + 1);
//             subset_scan_front->angle_min = latest_scan2_->angle_min + start_index_front * latest_scan2_->angle_increment;
//             subset_scan_front->angle_max = latest_scan2_->angle_min + end_index_front * latest_scan2_->angle_increment;

//             // Get the segment midpoints from cylinderPtr_
//             auto midpoints = cylinderPtr_->getSegmentMidpoints(subset_scan_front);

//             // Publish a marker for each midpoint
//             for (const auto &midpoint : midpoints)
//             {
//                 visualization_msgs::msg::Marker marker = produceMarkerPlant(midpoint);
//                 marker_pub_->publish(marker);
//                 RCLCPP_INFO(this->get_logger(), "Plant segment detected and marked at x: %.2f, y: %.2f", midpoint.x, midpoint.y);
//             }
//         }
//     }

//     // Publish the result
//     auto message = std_msgs::msg::Bool();
//     message.data = is_green;
//     publisher_->publish(message);

//     performed_scan_ = true;
// }

void Main::scanner()
{
    static auto last_scan_time = this->get_clock()->now();

    if ((this->get_clock()->now() - last_scan_time).seconds() < 1.0) {
        return;
    }
    last_scan_time = this->get_clock()->now();

    // Process image data to check if plant is yellow
    cv::Mat frame = latest_frame_;
    int x_start = frame.cols / 3;
    int x_end = 2 * frame.cols / 3;
    cv::Mat center_frame = frame(cv::Range::all(), cv::Range(x_start, x_end));
    cv::Mat hsv_frame;
    cv::cvtColor(center_frame, hsv_frame, cv::COLOR_BGR2HSV);

    // cv::Scalar lower_green(35, 50, 50), upper_green(85, 255, 255);
    // cv::Scalar lower_yellow(20, 100, 100), upper_yellow(30, 255, 255);

    cv::Scalar lower_green(35, 50, 50), upper_green(85, 255, 255);
    cv::Scalar lower_yellow(25, 150, 150), upper_yellow(35, 255, 255); // Adjusted yellow bounds


    cv::Mat green_mask, yellow_mask;
    cv::inRange(hsv_frame, lower_green, upper_green, green_mask);
    cv::inRange(hsv_frame, lower_yellow, upper_yellow, yellow_mask);

    int green_count = cv::countNonZero(green_mask);
    int yellow_count = cv::countNonZero(yellow_mask);

    bool is_green = green_count > yellow_count;
    if (is_green) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Current reading: Predominantly Green");
    } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Current reading: Predominantly Yellow");

        auto plant = cylinderPtr_->getSegmentMidpoints(latest_scan2_);

        RCLCPP_INFO(this->get_logger(), "gotten passed getsegmentmidpoints");
            visualization_msgs::msg::Marker marker = produceMarkerPlant(plant);
            marker_pub_->publish(marker);
            RCLCPP_INFO(this->get_logger(), "Plant segment detected and marked at x: %.2f, y: %.2f", plant.x, plant.y);
    }

    // auto message = std_msgs::msg::Bool();
    // message.data = is_green;
    // publisher_->publish(message);

    performed_scan_ = true;
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

void Main::imagecallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    latest_frame_ = cv_bridge::toCvCopy(msg, "bgr8")->image;  // Convert and store in latest_frame_
}

visualization_msgs::msg::Marker Main::produceMarkerPlant(geometry_msgs::msg::Point pt)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "cylinder_markers";
    marker.id = ct_++;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = pt;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.3; // Adjust the cylinder diameter
    marker.scale.y = 0.3; // Adjust the cylinder diameter
    marker.scale.z = 0.2; // Adjust the cylinder height
    marker.color.a = 1.0; // Transparency (1.0 is fully opaque)
    marker.color.r = 1.0; // Red color
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    return marker;
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
