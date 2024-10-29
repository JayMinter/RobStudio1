#include "cylinder.hpp"

Cylinder::Cylinder() : Node("cylinder_Node"), running_(true), completed_circle_(false), odometry_received_(false)
{
    // Define separate QoS settings for sensor data subscriptions
    rclcpp::QoS sensor_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    sensor_qos.keep_last(10).best_effort().durability_volatile();

    // Define slightly more robust QoS settings for publishers
    rclcpp::QoS publisher_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    publisher_qos.keep_last(10).reliable();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", sensor_qos, std::bind(&Cylinder::odomCallback, this, std::placeholders::_1));

    // Apply QoS settings to publishers (reliable, transient)
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/visualization_marker", publisher_qos);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", publisher_qos);
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", publisher_qos);
    detection_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/cylinder_detected", publisher_qos);

    // Start the cylinder detection thread
    // thread_ = std::make_unique<std::thread>(&Cylinder::detectCylinder, this);
}

Cylinder::~Cylinder()
{
    // Ensure the thread stops when the node is shut down
    running_ = false;
}

void Cylinder::processlatestScan(const sensor_msgs::msg::LaserScan::SharedPtr &scan_data)
{
    if (!scan_data)
    {
        RCLCPP_WARN(this->get_logger(), "Received null scan_data in processlatestScan.");
        return;
    }

    int smoothing_window_size = 4;
    auto smoothed_ranges = smoothScanData(*scan_data, smoothing_window_size);
    auto smoothed_scan = *scan_data;
    smoothed_scan.ranges = smoothed_ranges;

    RCLCPP_INFO(this->get_logger(), "processlatestScan triggered with new scan data.");
    countSegments(std::make_shared<sensor_msgs::msg::LaserScan>(smoothed_scan));
}

void Cylinder::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    currentOdom_ = *odom;
    odometry_received_ = true;

    // RCLCPP_INFO(this->get_logger(), "Updated odom: position (%.2f, %.2f)", odom->pose.pose.position.x, odom->pose.pose.position.y);
    // RCLCPP_INFO(this->get_logger(), "current odom: position (%.2f, %.2f)", currentOdom_.pose.pose.position.x, currentOdom_.pose.pose.position.y);
}

bool Cylinder::odomreceived(){
    return odometry_received_;
}

nav_msgs::msg::Odometry Cylinder::getOdometry(void)
{

    nav_msgs::msg::Odometry pose = currentOdom_;
    return pose;
}

bool Cylinder::detectCylinder(const sensor_msgs::msg::LaserScan::SharedPtr &scan_data)
{
    bool detected = false;

    // processlatestScan(scan_data);
    countSegments(scan_data);

    std::lock_guard<std::mutex> lock(segmentMutex_);

    for (auto &segment : segmentVector_)
    {
        double targetRadius = 0.15; // Expected radius of the cylinder (in meters)
        double tolerance = 0.075;   // Tolerance for detecting a cylinder

        const auto &p1 = segment.front();
        const auto &p2 = segment.back();

        double d = sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
        // RCLCPP_INFO(this->get_logger(), "distance between segment front and back is %.2f ", d);

        // Check if segment length is valid and differs significantly from target radius
        if (segment.size() >= 6 && (fabs(d - 2 * targetRadius) < tolerance))
        {
            geometry_msgs::msg::Point centre = findCentre(segment.front(), segment.back(), targetRadius);

            bool isValidCylinder = true; // Assume the segment forms a valid cylinder unless proven otherwise

            double tolerance2 = 0.02;
            for (const auto &point : segment)
            {
                double centreD = sqrt((centre.x - point.x) * (centre.x - point.x) + (centre.y - point.y) * (centre.y - point.y));

                // RCLCPP_INFO(this->get_logger(), "distance from centre is %.2f ", centreD);
                // If distance from center differs too much from target radius, invalidate this cylinder
                if (fabs(centreD - targetRadius) > tolerance2)
                {
                    isValidCylinder = false; // Not a valid cylinder, exit loop
                    break;
                }
            }

            // Only proceed to add the marker if it's a valid cylinder
            if (isValidCylinder)
            {
                if (!checkExisting(centre) || firstCent)
                {
                    firstCent = false;
                    centres.push_back(centre);
                    if (centres.size() != 0)
                    {
                        if (!std::isnan(centre.x) && !std::isnan(centre.y) && !std::isnan(centre.z))
                        {
                            RCLCPP_INFO(this->get_logger(), "Cylinder with radius ~%.2fm detected. Center: x = %.2f, y = %.2f, z = %.2f", targetRadius, centre.x, centre.y, centre.z);
                            visualization_msgs::msg::Marker marker = produceMarkerCylinder(centre);
                            marker_pub_->publish(marker);
                            cylinder_centre_ = centre;

                            // cylinder_centre_.x = (centre.x - 0.5);
                            sendMessage();
                            // publishCircleMotion(centre);
                            detected = true;
                        }
                    }
                }
            }
        }
    }
    return detected;
}

void Cylinder::sendMessage(){
                                std_msgs::msg::Bool detected_msg;
                            detected_msg.data = true;
                            detection_publisher_->publish(detected_msg);
}





// void Cylinder::publishCircleMotion(geometry_msgs::msg::Point cylinder)
// {
//     double tolerance = 0.2;                               // Tolerance for reaching the target distance
//     double targetDistance = 0.5;                          // Desired distance from the cylinder to start circling
//     double circlingSpeed = 0.15;                          // Linear speed to maintain while circling
//     double angularSpeed = circlingSpeed / targetDistance; // Angular speed for a circular path
//     bool atCircle = false;                                // Flag to indicate arrival at target distance

//     // Step 1: Controlled approach to the cylinder
//     RCLCPP_INFO(this->get_logger(), "Starting approach to cylinder.");

//     // while (rclcpp::ok() && !atCircle)
//     // {
//     //     // Calculate current distance to the cylinder
//     //     float distanceToCylinder = sqrt(pow(getOdometry().pose.pose.position.x - cylinder.x, 2) +
//     //                                     pow(getOdometry().pose.pose.position.y - cylinder.y, 2));

//     //     // Check if the robot is within the target distance
//     //     if (fabs(distanceToCylinder - targetDistance) <= tolerance)
//     //     {
//     //         atCircle = true; // Robot has reached the target distance
//     //         RCLCPP_INFO(this->get_logger(), "Reached target distance. Stopping and aligning perpendicularly.");
//     //         // stopTurtle(); // Stop movement before orienting
//     //         break;
//     //     }

//     //     // Adjust speed based on the distance to avoid overshooting
//     //     // double approachSpeed = std::min(0.26, 0.5 * (distanceToCylinder - targetDistance)); // Scale down speed as it gets closer
//     //     double angle = computeSteering(getOdometry(), cylinder);                            // Calculate angle to cylinder

//     //     RCLCPP_INFO(this->get_logger(), "Angle to Cylinder: %.2f", angle);
//     //     RCLCPP_INFO(this->get_logger(), "Distance to Cylinder: %.2f", distanceToCylinder);

//     //     // Move towards the cylinder with scaled speed
//     //     // publishCommands(approachSpeed, angle * 0.5); // Adjust the angle scaling as needed
//     //     // rclcpp::Rate(10).sleep();                    // Small delay to stabilize control
//     // }

//     // if (!atCircle)
//     // {
//     //     RCLCPP_ERROR(this->get_logger(), "Failed to approach cylinder. Exiting circling motion.");
//     //     return; // Exit if we couldn't reach the target distance
//     // }

//     // Step 2: Align perpendicular to the cylinder
//     double perpendicularAngle = computePerpendicularAngle(getOdometry(), cylinder);
//     while (fabs(perpendicularAngle) > tolerance && rclcpp::ok())
//     {
//         perpendicularAngle = computePerpendicularAngle(getOdometry(), cylinder);
//         RCLCPP_INFO(this->get_logger(), "Adjusting to perpendicular. Angle: %.2f", perpendicularAngle);

//         publishCommands(0.0, 0.5 * perpendicularAngle); // Rotate on the spot to align
//         // rclcpp::Rate(10).sleep();                       // Allow for smooth control
//     }
//     stopTurtle();
//     nav_msgs::msg::Odometry startCircle = getOdometry();
//     RCLCPP_INFO(this->get_logger(), "Aligned perpendicular to the cylinder. Starting circling motion.");

//     publishCommands(circlingSpeed, angularSpeed);
//     std::this_thread::sleep_for(std::chrono::seconds(2));

//     double distanceToStart = sqrt(pow(getOdometry().pose.pose.position.x - startCircle.pose.pose.position.x, 2) + pow(getOdometry().pose.pose.position.y - startCircle.pose.pose.position.y, 2));

//     while(distanceToStart > 0.05){
//         publishCommands(circlingSpeed, angularSpeed);
//         distanceToStart = sqrt(pow(getOdometry().pose.pose.position.x - startCircle.pose.pose.position.x, 2) + pow(getOdometry().pose.pose.position.y - startCircle.pose.pose.position.y, 2));
//     }
//     if(distanceToStart < 0.05){
//     stopTurtle();
//     std_msgs::msg::Bool detecting;
//     detecting.data = false;
//     detection_publisher_->publish(detecting);
//     RCLCPP_INFO(this->get_logger(), "Completed circling the cylinder.");
//     }
// }


// void Cylinder::publishCircleMotion(geometry_msgs::msg::Point cylinder)
// {
//     double tolerance = 0.2;                               // Tolerance for reaching the target distance
//     double targetDistance = 0.5;                          // Desired distance from the cylinder to start circling
//     double circlingSpeed = 0.15;                          // Linear speed to maintain while circling
//     double angularSpeed = circlingSpeed / targetDistance; // Angular speed for a circular path

//     // Step 1: Controlled approach to the cylinder
//     RCLCPP_INFO(this->get_logger(), "Starting approach to cylinder.");

//     double perpendicularAngle = computePerpendicularAngle(getOdometry(), cylinder);
//     while (fabs(perpendicularAngle) > tolerance && rclcpp::ok())
//     {
//         perpendicularAngle = computePerpendicularAngle(getOdometry(), cylinder);
//         RCLCPP_INFO(this->get_logger(), "Adjusting to perpendicular. Angle: %.2f", perpendicularAngle);

//         publishCommands(0.0, 0.5 * perpendicularAngle); // Rotate on the spot to align
//         rclcpp::Rate(10).sleep();                       // Allow for smooth control
//     }
//     stopTurtle();
//     nav_msgs::msg::Odometry startCircle = getOdometry();
//     RCLCPP_INFO(this->get_logger(), "Aligned perpendicular to the cylinder. Starting circling motion.");

//     // Step 2: Begin circling the cylinder
//     publishCommands(circlingSpeed, angularSpeed);

//     // Check distance to start position for a complete loop
//     double distanceToStart = sqrt(pow(getOdometry().pose.pose.position.x - startCircle.pose.pose.position.x, 2) +
//                                   pow(getOdometry().pose.pose.position.y - startCircle.pose.pose.position.y, 2));

//     while (distanceToStart > 0.05 && rclcpp::ok())
//     {
//         publishCommands(circlingSpeed, angularSpeed);
//         distanceToStart = sqrt(pow(getOdometry().pose.pose.position.x - startCircle.pose.pose.position.x, 2) +
//                                pow(getOdometry().pose.pose.position.y - startCircle.pose.pose.position.y, 2));
//         rclcpp::Rate(10).sleep(); // Allow for smooth circling
//     }

//     // Step 3: Stop and send a completion message
//     if (distanceToStart <= 0.05)
//     {
//         stopTurtle();
//         std_msgs::msg::Bool detecting;
//         detecting.data = false;
//         detection_publisher_->publish(detecting);
//         RCLCPP_INFO(this->get_logger(), "Completed circling the cylinder.");
//     }
// }

void Cylinder::publishCircleMotion(geometry_msgs::msg::Point cylinder)
{
    completed_circle_ = false;
    double targetDistance = 0.4;                          // Desired distance from the cylinder to start circling
    double circlingSpeed = 0.15;                          // Linear speed to maintain while circling
    double angularSpeed = circlingSpeed / targetDistance; // Angular speed for a circular path
    double approachSpeed = 0.1;                           // Speed for controlled approach
    double tolerance = 0.05;                              // Tolerance for distance maintenance
    double minDistanceBeforeChecking = 0.2;               // Minimum distance to travel before checking for completion

    RCLCPP_INFO(this->get_logger(), "Starting approach to cylinder.");

    // Step 1: Approach the cylinder to the target distance
    while (rclcpp::ok() && getDistanceToCylinder(cylinder) > targetDistance + tolerance)
    {
        publishCommands(approachSpeed, 0.0); // Move straight toward the cylinder
        rclcpp::Rate(10).sleep();
    }

    stopTurtle();
    RCLCPP_INFO(this->get_logger(), "Reached target distance from the cylinder. Aligning to perpendicular.");

    // Step 2: Align perpendicular to the cylinder
    double perpendicularAngle = computePerpendicularAngle(getOdometry(), cylinder);
    while (rclcpp::ok() && fabs(perpendicularAngle) > 0.1) // Allow small angular error tolerance
    {
        perpendicularAngle = computePerpendicularAngle(getOdometry(), cylinder);
        publishCommands(0.0, 0.5 * perpendicularAngle); // Rotate on the spot to align
    }
    stopTurtle();
    RCLCPP_INFO(this->get_logger(), "Aligned perpendicular to the cylinder. Starting circling motion.");

    // Step 3: Start circling motion and record the starting position
    auto startCirclePosition = getOdometry().pose.pose.position;
    bool isFarEnoughFromStart = false;

    // Step 4: Circle around the cylinder while maintaining distance
    while (rclcpp::ok())
    {
        double currentDistance = getDistanceToCylinder(cylinder);
        
        // Adjust circling speed based on distance to maintain target distance
        if (currentDistance > targetDistance + tolerance)
        {
            publishCommands(circlingSpeed, angularSpeed + 0.05); // Adjust inward
        }
        else if (currentDistance < targetDistance - tolerance)
        {
            publishCommands(circlingSpeed, angularSpeed - 0.05); // Adjust outward
        }
        else
        {
            publishCommands(circlingSpeed, angularSpeed); // Maintain current path
        }

        // Check if the robot is far enough from the start position before allowing completion check
        auto currentPosition = getOdometry().pose.pose.position;
        double distanceFromStart = sqrt(pow(currentPosition.x - startCirclePosition.x, 2) +
                                        pow(currentPosition.y - startCirclePosition.y, 2));

        if (distanceFromStart > minDistanceBeforeChecking)
        {
            isFarEnoughFromStart = true; // Enable the completion check once far enough
        }

        // Check if the robot has completed one full circle after moving away sufficiently
        if (isFarEnoughFromStart && distanceFromStart < 0.1) // Threshold to complete one circle
        {
            stopTurtle();
            RCLCPP_INFO(this->get_logger(), "Completed one full rotation around the cylinder.");
            completed_circle_ = true;
            break;
        }

        rclcpp::Rate(10).sleep(); // Control rate for smooth motion
    }
}


bool Cylinder::getCompletedCircle(){
    return completed_circle_;
}

double Cylinder::getDistanceToCylinder(geometry_msgs::msg::Point cylinder)
{
    auto currentPosition = getOdometry().pose.pose.position;
    return sqrt(pow(cylinder.x - currentPosition.x, 2) + pow(cylinder.y - currentPosition.y, 2));
}

double Cylinder::computePerpendicularAngle(nav_msgs::msg::Odometry currentOdom, geometry_msgs::msg::Point target)
{
    auto currentPosition = currentOdom.pose.pose.position;
    double dx = target.x - currentPosition.x;
    double dy = target.y - currentPosition.y;
    double angleToCylinder = atan2(dy, dx);

    // Get robot's current heading
    double robotHeading = tf2::getYaw(currentOdom.pose.pose.orientation);

    // Calculate perpendicular angle (90 degrees offset)
    return angleToCylinder - robotHeading - M_PI_2; // Adjust to be perpendicular
}


// void Cylinder::publishCircleMotion(geometry_msgs::msg::Point cylinder)
// {
//     RCLCPP_INFO(this->get_logger(), "Simplified publishCircleMotion called.");
//     stopTurtle();
// rclcpp::sleep_for(std::chrono::seconds(3));
//         std_msgs::msg::Bool detecting;
//     detecting.data = false;
//     detection_publisher_->publish(detecting);
//     // No operations - just a log statement to observe behavior
// }
geometry_msgs::msg::Point Cylinder::getCylinder()
{

    return cylinder_centre_;
}

// double Cylinder::computePerpendicularAngle(const nav_msgs::msg::Odometry &odom, geometry_msgs::msg::Point goal)
// {
//     // Calculate the direction to the goal (cylinder center)
//     double dx = goal.x - odom.pose.pose.position.x;
//     double dy = goal.y - odom.pose.pose.position.y;
//     double angleToGoal = atan2(dy, dx);

//     // Calculate current yaw (orientation) of the robot
//     double currentYaw = tf2::getYaw(odom.pose.pose.orientation);

//     // Calculate the perpendicular angle (90 degrees to the right of the line to the goal)
//     double perpendicularAngle = angleToGoal - currentYaw + M_PI_2; // M_PI_2 is 90 degrees in radians

//     // Normalize to [-pi, pi] range
//     while (perpendicularAngle > M_PI)
//         perpendicularAngle -= 2.0 * M_PI;
//     while (perpendicularAngle < -M_PI)
//         perpendicularAngle += 2.0 * M_PI;

//     return perpendicularAngle;
// }

void Cylinder::publishCommands(double x, double z)
{
    geometry_msgs::msg::Twist move;
    move.linear.x = x;
    move.angular.z = z;

    cmd_vel_pub_->publish(move);
    // std::this_thread::sleep_for(std::chrono::seconds(2));
}

void Cylinder::stopTurtle()
{
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_cmd);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    // RCLCPP_INFO(this->get_logger(), "Robot has stopped before making circular motion.");
}

void Cylinder::countSegments(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    if (!scan)
        return;
    std::lock_guard<std::mutex> lock(segmentMutex_);
    geometry_msgs::msg::Point current, previous;

    std::vector<geometry_msgs::msg::Point> currentSegment;

    bool segStarted = false;

    for (size_t i = 1; i < scan->ranges.size(); i++)
    {
        if (std::isinf(scan->ranges.at(i - 1)) || std::isnan(scan->ranges.at(i - 1)))
        {
            continue;
        }

        while (i < scan->ranges.size() && (!std::isnan(scan->ranges.at(i))) && (scan->ranges.at(i) < scan->range_max))
        {
            float previousAngle = scan->angle_min + scan->angle_increment * (i - 1);
            previous.x = scan->ranges.at(i - 1) * cos(previousAngle);
            previous.y = scan->ranges.at(i - 1) * sin(previousAngle);
            previous.z = 0;

            float currentAngle = scan->angle_min + scan->angle_increment * i;
            current.x = scan->ranges.at(i) * cos(currentAngle);
            current.y = scan->ranges.at(i) * sin(currentAngle);
            current.z = 0;

            float dist = hypot((current.x - previous.x), (current.y - previous.y));

            if (dist < 0.075)
            {
                if (!segStarted)
                {
                    segStarted = true;
                    currentSegment.clear(); // Start a new segment
                }
                currentSegment.push_back(localToGlobal(currentOdom_, current));
            }
            else
            {
                if (segStarted && !currentSegment.empty())
                {
                    segmentVector_.push_back(currentSegment);
                    // RCLCPP_INFO(this->get_logger(), "Segment found with %ld points", currentSegment.size());
                    segStarted = false;
                }
                break;
            }

            i++;
        }

        if (segStarted && !currentSegment.empty())
        {
            segmentVector_.push_back(currentSegment);
            // detectCylinder(currentSegment);
            segStarted = false;
        }
    }

    // return segmentVector;
}



geometry_msgs::msg::Point Cylinder::findCentre(geometry_msgs::msg::Point P1, geometry_msgs::msg::Point P2, double r)
{
    geometry_msgs::msg::Point centre;

    double mx = (P1.x + P2.x) / 2.0;
    double my = (P1.y + P2.y) / 2.0;

    double d = sqrt((P2.x - P1.x) * (P2.x - P1.x) + (P2.y - P1.y) * (P2.y - P1.y));
    double h = sqrt(pow(r, 2) - pow(d / 2, 2));

    double dx = -(P2.y - P1.y) / d;
    double dy = (P2.x - P1.x) / d;

    double c1x = mx + h * dx;
    double c1y = my + h * dy;
    double c2x = mx - h * dx;
    double c2y = my - h * dy;

    double d2 = sqrt((currentOdom_.pose.pose.position.x - c1x) * (currentOdom_.pose.pose.position.x - c1x) + (currentOdom_.pose.pose.position.y - c1y) * (currentOdom_.pose.pose.position.y - c1y));
    double d3 = sqrt((currentOdom_.pose.pose.position.x - c2x) * (currentOdom_.pose.pose.position.x - c2x) + (currentOdom_.pose.pose.position.y - c2y) * (currentOdom_.pose.pose.position.y - c2y));

    if (d2 > d3)
    {
        centre.x = c1x;
        centre.y = c1y;
        centre.z = currentOdom_.pose.pose.position.z;
    }
    else
    {
        centre.x = c2x;
        centre.y = c2y;
        centre.z = currentOdom_.pose.pose.position.z;
    }

    return centre;
}

double Cylinder::computeSteering(nav_msgs::msg::Odometry origin, geometry_msgs::msg::Point goal)
{
    // Calculate the difference in position
    double dx = goal.x - origin.pose.pose.position.x;
    double dy = goal.y - origin.pose.pose.position.y;

    // Desired angle to the goal (cylinder) using atan2
    double desired_angle = atan2(dy, dx);

    // Current orientation of the robot (yaw)
    double current_yaw = tf2::getYaw(origin.pose.pose.orientation);

    // Calculate the angle difference between where the robot is facing and where it needs to face
    double steering_angle = desired_angle - current_yaw;

    // Normalize the angle to the range [-pi, pi] to avoid large turns
    while (steering_angle > M_PI)
        steering_angle -= 2.0 * M_PI;
    while (steering_angle < -M_PI)
        steering_angle += 2.0 * M_PI;

    return steering_angle;
}

std::vector<float> Cylinder::smoothScanData(const sensor_msgs::msg::LaserScan scan, int window_size)
{
    std::vector<float> smoothed_ranges(scan.ranges.size(), 0.0);
    int half_window = window_size / 2;

    for (size_t i = 0; i < scan.ranges.size(); ++i)
    {
        int count = 0;
        float sum = 0.0;

        for (int j = -half_window; j <= half_window; ++j)
        {
            int index = i + j;

            if (index >= 0 && index < scan.ranges.size() && !std::isnan(scan.ranges[index]) && !std::isinf(scan.ranges[index]))
            {
                sum += scan.ranges[index];
                count++;
            }
        }

        if (count > 0)
        {
            smoothed_ranges[i] = sum / count; // Calculate the average
        }
        else
        {
            smoothed_ranges[i] = scan.ranges[i]; // If no valid neighbors, use the original value
        }
    }

    return smoothed_ranges;
}

bool Cylinder::checkExisting(geometry_msgs::msg::Point centre)
{
    bool is_near_existing_center = false;

    for (const auto &existing_centre : centres)
    {
        double distance = sqrt(pow(centre.x - existing_centre.x, 2) + pow(centre.y - existing_centre.y, 2));
        if (distance < 1)
        {
            is_near_existing_center = true;
            break;
        }
    }

    return is_near_existing_center;
}

geometry_msgs::msg::Point Cylinder::localToGlobal(const nav_msgs::msg::Odometry &global, const geometry_msgs::msg::Point &local)
{
    geometry_msgs::msg::Point pt;

    pt.x = global.pose.pose.position.x + (local.x * cos(tf2::getYaw(global.pose.pose.orientation))) - (local.y * sin(tf2::getYaw(global.pose.pose.orientation)));
    pt.y = global.pose.pose.position.y + (local.x * sin(tf2::getYaw(global.pose.pose.orientation))) + (local.y * cos(tf2::getYaw(global.pose.pose.orientation)));
    pt.z = global.pose.pose.position.z;

    return pt;
}

visualization_msgs::msg::Marker Cylinder::produceMarkerCylinder(geometry_msgs::msg::Point pt)
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
    marker.color.r = 0.0; // Red color
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    return marker;
}


