#include "cylinder.hpp"

Cylinder::Cylinder() : Node("cylinder_Node"), running_(true), odometry_received_(false)
{
    // Define separate QoS settings for sensor data subscriptions
    rclcpp::QoS sensor_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    sensor_qos.keep_last(10).best_effort().durability_volatile();

    // Define slightly more robust QoS settings for publishers
    rclcpp::QoS publisher_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    publisher_qos.keep_last(10).reliable();

    // Apply QoS settings to subscriptions (best-effort, volatile)
    // scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    //     "/scan", sensor_qos, std::bind(&Cylinder::scanCallback, this, std::placeholders::_1));
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

// void Cylinder::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
// {
//     {
//         std::lock_guard<std::mutex> lock(scan_mutex_);
//         latest_scan_ = *scan;
//     }

//     // Log some information about the scan
//     RCLCPP_INFO(this->get_logger(), "Received LaserScan with %ld points", scan->ranges.size());

//     // Check if ranges contain valid data
//     int valid_count = 0;
//     for (auto range : scan->ranges)
//     {
//         if (!std::isnan(range) && !std::isinf(range))
//             valid_count++;
//     }
//     RCLCPP_INFO(this->get_logger(), "Number of valid points in scan: %d", valid_count);
// }

// void Cylinder::processlatestScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan_data)
// {
//     // Define a smoothing window size for the scan data
//     int smoothing_window_size = 5; // You can adjust this value based on the amount of smoothing needed

//     // Smooth the input scan data
//     auto smoothed_ranges = smoothScanData(*scan_data, smoothing_window_size);

//     // Create a modified copy of the input scan data with smoothed ranges
//     auto smoothed_scan = *scan_data;
//     smoothed_scan.ranges = smoothed_ranges;

//     RCLCPP_INFO(this->get_logger(), "processlatestScan triggered with new scan data.");

//     // Process segments using the smoothed scan data
//     countSegments(std::make_shared<sensor_msgs::msg::LaserScan>(smoothed_scan));
// }

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

                            cylinder_centre_.x = (centre.x - 1.0);
                            std_msgs::msg::Bool detected_msg;
                            detected_msg.data = true;
                            detection_publisher_->publish(detected_msg);
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




// void Cylinder::publishCircleMotion(geometry_msgs::msg::Point cylinder)
// {
//     double tolerance = 0.2;

//     float distanceToCylinder = sqrt(pow(getOdometry().pose.pose.position.x - cylinder.x, 2) +
//                                     pow(getOdometry().pose.pose.position.y - cylinder.y, 2));
//     // float initialPos = distanceToCylinder;

//     // stopTurtle(); // Stops the robot

//     while(fabs(distanceToCylinder - 1.2) > tolerance)
//     {
//         // std::this_thread::sleep_for(std::chrono::seconds(1));
//         distanceToCylinder = sqrt(pow(getOdometry().pose.pose.position.x - cylinder.x, 2) +
//                                   pow(getOdometry().pose.pose.position.y - cylinder.y, 2));

//         double angle = computeSteering(getOdometry(), cylinder); // Recompute the angle
//         RCLCPP_INFO(this->get_logger(), "Angle to Cylinder is %.2f", angle);
//         RCLCPP_INFO(this->get_logger(), "Distance to Cylinder is %.2f", distanceToCylinder);
//         publishCommands(0.26, (0.5*angle));
//         // RCLCPP_INFO(this->get_logger(), "Robot is adjusting its path towards the cylinder.");
//     }

//     // Once the robot reaches the desired distance, stop the motion
//     stopTurtle();

//     // publishCommands(0.26, 0.26*angle);
//     // RCLCPP_INFO(this->get_logger(), "Robot reached the target distance. Stopping.");
// }

void Cylinder::publishCircleMotion(geometry_msgs::msg::Point cylinder)
{
    double tolerance = 0.2;                               // Tolerance for reaching the target distance
    double targetDistance = 0.5;                          // Desired distance from the cylinder to start circling
    double circlingSpeed = 0.15;                          // Linear speed to maintain while circling
    double angularSpeed = circlingSpeed / targetDistance; // Angular speed for a circular path
    bool atCircle = false;                                // Flag to indicate arrival at target distance

    // Step 1: Controlled approach to the cylinder
    RCLCPP_INFO(this->get_logger(), "Starting approach to cylinder.");

    while (rclcpp::ok() && !atCircle)
    {
        // Calculate current distance to the cylinder
        float distanceToCylinder = sqrt(pow(getOdometry().pose.pose.position.x - cylinder.x, 2) +
                                        pow(getOdometry().pose.pose.position.y - cylinder.y, 2));

        // Check if the robot is within the target distance
        if (fabs(distanceToCylinder - targetDistance) <= tolerance)
        {
            atCircle = true; // Robot has reached the target distance
            RCLCPP_INFO(this->get_logger(), "Reached target distance. Stopping and aligning perpendicularly.");
            // stopTurtle(); // Stop movement before orienting
            break;
        }

        // Adjust speed based on the distance to avoid overshooting
        // double approachSpeed = std::min(0.26, 0.5 * (distanceToCylinder - targetDistance)); // Scale down speed as it gets closer
        double angle = computeSteering(getOdometry(), cylinder);                            // Calculate angle to cylinder

        RCLCPP_INFO(this->get_logger(), "Angle to Cylinder: %.2f", angle);
        RCLCPP_INFO(this->get_logger(), "Distance to Cylinder: %.2f", distanceToCylinder);

        // Move towards the cylinder with scaled speed
        // publishCommands(approachSpeed, angle * 0.5); // Adjust the angle scaling as needed
        // rclcpp::Rate(10).sleep();                    // Small delay to stabilize control
    }

    if (!atCircle)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to approach cylinder. Exiting circling motion.");
        return; // Exit if we couldn't reach the target distance
    }

    // Step 2: Align perpendicular to the cylinder
    double perpendicularAngle = computePerpendicularAngle(getOdometry(), cylinder);
    while (fabs(perpendicularAngle) > tolerance && rclcpp::ok())
    {
        perpendicularAngle = computePerpendicularAngle(getOdometry(), cylinder);
        RCLCPP_INFO(this->get_logger(), "Adjusting to perpendicular. Angle: %.2f", perpendicularAngle);

        publishCommands(0.0, 0.5 * perpendicularAngle); // Rotate on the spot to align
        // rclcpp::Rate(10).sleep();                       // Allow for smooth control
    }
    stopTurtle();
    nav_msgs::msg::Odometry startCircle = getOdometry();
    RCLCPP_INFO(this->get_logger(), "Aligned perpendicular to the cylinder. Starting circling motion.");

    publishCommands(circlingSpeed, angularSpeed);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    double distanceToStart = sqrt(pow(getOdometry().pose.pose.position.x - startCircle.pose.pose.position.x, 2) + pow(getOdometry().pose.pose.position.y - startCircle.pose.pose.position.y, 2));

    while(distanceToStart > 0.05){
        publishCommands(circlingSpeed, angularSpeed);
        distanceToStart = sqrt(pow(getOdometry().pose.pose.position.x - startCircle.pose.pose.position.x, 2) + pow(getOdometry().pose.pose.position.y - startCircle.pose.pose.position.y, 2));
    }
    if(distanceToStart < 0.05){
    stopTurtle();
    std_msgs::msg::Bool detecting;
    detecting.data = false;
    detection_publisher_->publish(detecting);
    RCLCPP_INFO(this->get_logger(), "Completed circling the cylinder.");
    }
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

double Cylinder::computePerpendicularAngle(const nav_msgs::msg::Odometry &odom, geometry_msgs::msg::Point goal)
{
    // Calculate the direction to the goal (cylinder center)
    double dx = goal.x - odom.pose.pose.position.x;
    double dy = goal.y - odom.pose.pose.position.y;
    double angleToGoal = atan2(dy, dx);

    // Calculate current yaw (orientation) of the robot
    double currentYaw = tf2::getYaw(odom.pose.pose.orientation);

    // Calculate the perpendicular angle (90 degrees to the right of the line to the goal)
    double perpendicularAngle = angleToGoal - currentYaw + M_PI_2; // M_PI_2 is 90 degrees in radians

    // Normalize to [-pi, pi] range
    while (perpendicularAngle > M_PI)
        perpendicularAngle -= 2.0 * M_PI;
    while (perpendicularAngle < -M_PI)
        perpendicularAngle += 2.0 * M_PI;

    return perpendicularAngle;
}

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

// void Cylinder::countSegments(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
//     if (!scan) return;

//     std::lock_guard<std::mutex> lock(segmentMutex_);
//     segmentVector_.clear();

//     geometry_msgs::msg::Point current, previous;
//     std::vector<geometry_msgs::msg::Point> currentSegment;
//     bool segStarted = false;

//     for (size_t i = 1; i < scan->ranges.size(); i++) {
//         if (std::isinf(scan->ranges.at(i - 1)) || std::isnan(scan->ranges.at(i - 1)))
//             continue;

//         while (i < scan->ranges.size() && !std::isnan(scan->ranges.at(i)) && scan->ranges.at(i) < scan->range_max) {
//             float previousAngle = scan->angle_min + scan->angle_increment * (i - 1);
//             previous.x = scan->ranges.at(i - 1) * cos(previousAngle);
//             previous.y = scan->ranges.at(i - 1) * sin(previousAngle);
//             previous.z = 0;

//             float currentAngle = scan->angle_min + scan->angle_increment * i;
//             current.x = scan->ranges.at(i) * cos(currentAngle);
//             current.y = scan->ranges.at(i) * sin(currentAngle);
//             current.z = 0;

//             float dist = hypot((current.x - previous.x), (current.y - previous.y));

//             if (dist < 0.075) {
//                 if (!segStarted) {
//                     segStarted = true;
//                     currentSegment.clear();
//                 }
//                 currentSegment.push_back(localToGlobal(currentOdom_, current));
//             } else {
//                 if (segStarted && !currentSegment.empty()) {
//                     segmentVector_.push_back(currentSegment);
//                     segStarted = false;
//                 }
//                 break;
//             }
//             i++;
//         }

//         if (segStarted && !currentSegment.empty()) {
//             segmentVector_.push_back(currentSegment);
//             segStarted = false;
//         }
//     }
// }

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
            long unsigned int index = i + j;

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

geometry_msgs::msg::Point Cylinder::getSegmentMidpoints(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
{
    geometry_msgs::msg::Point plant_centre;

    // Define angle range for the front scan (e.g., -15 to 15 degrees)
    double angle_min_deg_front = -60.0;
    double angle_max_deg_front = 60.0;
    double angle_min_rad_front = angle_min_deg_front * M_PI / 180.0;
    double angle_max_rad_front = angle_max_deg_front * M_PI / 180.0;

    // Calculate index range for front scan based on the angle limits
    int start_index_front = static_cast<int>((angle_min_rad_front - scan->angle_min) / scan->angle_increment);
    int end_index_front = static_cast<int>((angle_max_rad_front - scan->angle_min) / scan->angle_increment);

    // Define the front subset of the scan data
    auto subset_scan_front = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
    subset_scan_front->ranges = std::vector<float>(scan->ranges.begin() + start_index_front, scan->ranges.begin() + end_index_front + 1);
    subset_scan_front->angle_min = scan->angle_min + start_index_front * scan->angle_increment;
    subset_scan_front->angle_max = scan->angle_min + end_index_front * scan->angle_increment;

    // Lock the mutex and count the segments in this front scan range
    {
        segmentVector_.clear();  // Clear previous segments
        countSegments(subset_scan_front);  // Populate segmentVector_ with new segments in front
        std::lock_guard<std::mutex> lock(segmentMutex_);

        // Check each segment to determine if it qualifies as a plant (width < 0.3m)
        for (const auto &segment : segmentVector_)
        {
            if (!segment.empty())
            {
                // Filter out segments with NaN values
                bool contains_nan = false;
                for (const auto &point : segment)
                {
                    if (std::isnan(point.x) || std::isnan(point.y))
                    {
                        contains_nan = true;
                        break;
                    }
                }
                if (contains_nan)
                {
                    continue; // Skip this segment if it contains NaN values
                }

                // Calculate the width of the segment (distance between first and last points)
                const auto &p1 = segment.front();
                const auto &p2 = segment.back();
                double segment_width = sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));

                // Check if the segment meets the criteria for a plant (width < 0.3m) and is more than 0.3m away from the odometry
                double distance_to_odometry = sqrt(pow(p1.x - currentOdom_.pose.pose.position.x, 2) + pow(p1.y - currentOdom_.pose.pose.position.y, 2));
                if (segment_width < 0.3 && distance_to_odometry > 0.3)
                {
                    // If valid, calculate the center of the segment using the existing `findCentre` function
                    plant_centre = findCentre(p1, p2, segment_width / 2.0);

                    RCLCPP_INFO(this->get_logger(), "Plant detected. Center: x = %.2f, y = %.2f, z = %.2f", plant_centre.x, plant_centre.y, plant_centre.z);
                    return plant_centre;  // Return the first plant center found
                }
            }
        }
    }

    plant_centre.x = std::numeric_limits<double>::quiet_NaN();
    plant_centre.y = std::numeric_limits<double>::quiet_NaN();
    plant_centre.z = std::numeric_limits<double>::quiet_NaN();

    RCLCPP_WARN(this->get_logger(), "No plant found in the specified scan range.");
    return plant_centre;
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

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<Cylinder>());
//     rclcpp::shutdown();
//     return 0;
// }
