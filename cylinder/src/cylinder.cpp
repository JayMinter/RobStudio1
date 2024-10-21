#include "cylinder.hpp"

Cylinder::Cylinder() : Node("cylinder_Node")
{
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Cylinder::scanCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&Cylinder::odomCallback, this, std::placeholders::_1));
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/visualization_marker", 10);
}

    void Cylinder::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Process segments and find cylinders
        auto segments = countSegments(scan);
    }

    void Cylinder::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        // Store the current odometry for transformations
        currentOdom = *odom;
    }

    std::vector<std::vector<geometry_msgs::msg::Point>> Cylinder::countSegments(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        geometry_msgs::msg::Point current, previous;
        std::vector<std::vector<geometry_msgs::msg::Point>> segmentVector;
        std::vector<geometry_msgs::msg::Point> currentSegment;

        bool segStarted = false;

        for (size_t i = 1; i < scan->ranges.size(); i++)
        {
            // Skip invalid ranges
            if (std::isinf(scan->ranges.at(i - 1)) || std::isnan(scan->ranges.at(i - 1)))
            {
                continue;
            }

            // Extract points to form segments
            while (i < scan->ranges.size() && (!std::isnan(scan->ranges.at(i))) && (scan->ranges.at(i) < scan->range_max))
            {
                // Calculate previous and current points
                float previousAngle = scan->angle_min + scan->angle_increment * (i - 1);
                previous.x = scan->ranges.at(i - 1) * cos(previousAngle);
                previous.y = scan->ranges.at(i - 1) * sin(previousAngle);
                previous.z = 0;

                float currentAngle = scan->angle_min + scan->angle_increment * i;
                current.x = scan->ranges.at(i) * cos(currentAngle);
                current.y = scan->ranges.at(i) * sin(currentAngle);
                current.z = 0;

                float dist = hypot((current.x - previous.x), (current.y - previous.y));

                // Check if the distance is small enough to add to the current segment
                if (dist < 0.075)
                {
                    if (!segStarted)
                    {
                        segStarted = true;
                        currentSegment.clear(); // Start a new segment
                    }
                    // Add the transformed point to the current segment
                    currentSegment.push_back(localToGlobal(currentOdom, current));
                }
                else
                {
                    // End the current segment if distance is too large
                    if (segStarted && !currentSegment.empty())
                    {
                        segmentVector.push_back(currentSegment);
                        if (!isThisAWall(currentSegment))
                        {
                            if (!isThisACorner(currentSegment))
                                detectCylinder(currentSegment);
                        }
                        segStarted = false; // Reset the segment flag
                    }
                    break; // Break out of the while loop
                }

                // Move to the next point
                i++;
            }

            // If the loop ends and a segment was started, add it to the segment vector
            if (segStarted && !currentSegment.empty())
            {
                segmentVector.push_back(currentSegment);
                rclcpp::sleep_for(std::chrono::milliseconds(10));
                if (!isThisAWall(currentSegment))
                {
                    if (!isThisACorner(currentSegment))
                        detectCylinder(currentSegment);
                }
                segStarted = false; // Reset the segment flag
            }
        }

        return segmentVector;
    }

    void Cylinder::detectCylinder(const std::vector<geometry_msgs::msg::Point> &segment)
    {

        if (segment.size() >= 6)
        {

            const auto &p1 = segment.front();
            const auto &p2 = segment.at(segment.size() / 2);
            const auto &p3 = segment.back();

            // Calculate the distances between points
            double a = hypot(p2.x - p1.x, p2.y - p1.y); // Distance p1-p2
            double b = hypot(p3.x - p2.x, p3.y - p2.y); // Distance p2-p3
            double c = hypot(p3.x - p1.x, p3.y - p1.y); // Distance p1-p3

            // Calculate the angle theta using the cosine rule
            double cosTheta = (a * a + b * b - c * c) / (2 * a * b);
            double theta = acos(cosTheta); // Angle formed at p2

            // Calculate radius R of the circle approximated by the parabola
            double R = c / (2 * fabs(sin(theta / 2)));

            // Check if the radius is approximately 0.3m
            double targetRadius = 0.15; // Desired circle radius
            double tolerance_ = 0.01;   // Tolerance for radius matching

            // Filter out circles with different radii
            if (fabs(R - targetRadius) < tolerance_)
            {
                geometry_msgs::msg::Point centre = findCentre(segment.front(), segment.back(), targetRadius);

                // If not near any existing center, add to the list and publish the marker
                RCLCPP_INFO(this->get_logger(), "Is this the first center %i, is there an existing center %i", firstCent, checkExisting(centre));
                if (!checkExisting(centre) || firstCent)
                {
                    firstCent = false;
                    centres.push_back(centre);
                    RCLCPP_INFO(this->get_logger(), "Circle with radius ~%.2fm detected. Center: x = %.2f, y = %.2f, z = %.2f", targetRadius, centre.x, centre.y, centre.z);

                    visualization_msgs::msg::Marker marker = produceMarkerCylinder(centre);
                    marker_pub_->publish(marker);
                }
            }
        }
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

        double d2 = sqrt((currentOdom.pose.pose.position.x - c1x) * (currentOdom.pose.pose.position.x - c1x) + (currentOdom.pose.pose.position.y - c1y) * (currentOdom.pose.pose.position.y - c1y));
        double d3 = sqrt((currentOdom.pose.pose.position.x - c2x) * (currentOdom.pose.pose.position.x - c2x) + (currentOdom.pose.pose.position.y - c2y) * (currentOdom.pose.pose.position.y - c2y));

        if (d2 > d3)
        {
            centre.x = c1x;
            centre.y = c1y;
            centre.z = currentOdom.pose.pose.position.z;
        }
        else
        {
            centre.x = c2x;
            centre.y = c2y;
            centre.z = currentOdom.pose.pose.position.z;
        }

        return centre;
    }

    bool Cylinder::checkExisting(geometry_msgs::msg::Point centre)
    {
        // RCLCPP_INFO(this->get_logger(), "Checked the tolerances for radius");
        RCLCPP_INFO(this->get_logger(), "Check against the other %ld existing circles", centres.size());
        // Find the center of the circle
        // bool firstCent = true;
        // Check if the center is within 0.5m of any existing centers
        bool is_near_existing_center = false;
        // if(firstCent){
        //     firstCent = false;
        // }else{
        for (const auto &existing_centre : centres)
        {

            RCLCPP_INFO(this->get_logger(), "Checking against circle at %.2f, %.2f", existing_centre.x, existing_centre.y);
            double distance = sqrt(pow(centre.x - existing_centre.x, 2) + pow(centre.y - existing_centre.y, 2));
            if (distance < 1)
            {
                is_near_existing_center = true;
                RCLCPP_INFO(this->get_logger(), "Circle is near another which is at %.2f, %.2f", existing_centre.x, existing_centre.y);
                break;
            }
        }
        // }

        // If not near any existing center, add to the list and publish the marker
        return is_near_existing_center;
    }

    void Cylinder::visualizeSegment(const std::vector<geometry_msgs::msg::Point> &segment)
    {
        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = this->get_clock()->now();
        line_marker.ns = "cylinder_markers";
        line_marker.id = ct_++;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.scale.x = 0.05; // Line width
        line_marker.color.a = 0.8;
        line_marker.color.r = 0.0;
        line_marker.color.g = 1.0; // Green color for line segments
        line_marker.color.b = 0.0;

        // Add points to the line strip marker
        for (const auto &point : segment)
        {
            line_marker.points.push_back(point);
        }

        marker_pub_->publish(line_marker);
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

        RCLCPP_INFO(this->get_logger(), "placed cylinder");
    }

    geometry_msgs::msg::Point Cylinder::localToGlobal(const nav_msgs::msg::Odometry &global, const geometry_msgs::msg::Point &local)
    {
        geometry_msgs::msg::Point pt;

        pt.x = global.pose.pose.position.x + (local.x * cos(tf2::getYaw(global.pose.pose.orientation))) - (local.y * sin(tf2::getYaw(global.pose.pose.orientation)));
        pt.y = global.pose.pose.position.y + (local.x * sin(tf2::getYaw(global.pose.pose.orientation))) + (local.y * cos(tf2::getYaw(global.pose.pose.orientation)));
        pt.z = global.pose.pose.position.z;

        return pt;
    }

    bool Cylinder::isThisAWall(const std::vector<geometry_msgs::msg::Point> &segmentVector)
    {
        bool isWall = true;

        float m = (segmentVector.back().y - segmentVector.front().y) /
                  (segmentVector.back().x - segmentVector.front().x);
        float b = segmentVector.front().y - (m * segmentVector.front().x);

        float tolerance = 0.05;

        for (const auto &point : segmentVector)
        {
            float x = point.x;
            float y = point.y;

            float distance = fabs(m * x - y + b) / sqrt(m * m + 1);

            if (distance > tolerance)
            {
                isWall = false;
                break;
            }
        }

        return isWall;
    }

    bool Cylinder::isThisACorner(const std::vector<geometry_msgs::msg::Point> &segmentVector)
    {
        // We need at least 3 points to calculate two gradients for corner detection
        if (segmentVector.size() < 3)
            return false;

        bool isCorner = false;
        float tolerance__ = 0.5; // Tolerance for the product to be roughly -1

        for (size_t i = 1; i < segmentVector.size() - 1; ++i)
        {
            // Calculate the gradient between the current point and the previous point
            float m1 = (segmentVector[i].y - segmentVector[i - 1].y) /
                       (segmentVector[i].x - segmentVector[i - 1].x);

            // Calculate the gradient between the current point and the next point
            float m2 = (segmentVector[i + 1].y - segmentVector[i].y) /
                       (segmentVector[i + 1].x - segmentVector[i].x);

            // Check if the product of the two gradients is approximately -1
            float gradientProduct = m1 * m2;

            if (gradientProduct >= -1.0 - tolerance__ && gradientProduct <= -1.0 + tolerance__)
            {
                isCorner = true;
                break;
            }
        }

        return isCorner;
    }


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Cylinder>());
    rclcpp::shutdown();
    return 0;
}

