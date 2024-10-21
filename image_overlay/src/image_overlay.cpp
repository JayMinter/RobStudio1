#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

class ImageOverlay : public rclcpp::Node
{
public:
    ImageOverlay()
        : Node("image_overlay_node")
    {
        // Load the background (PNG) and foreground (PGM) images
        background_ = cv::imread("/home/student/ros2_ws/src/Crop-Guard/image_overlay/images/farmWith3CropRows2.png", cv::IMREAD_UNCHANGED); // PNG image with RGBA channels
        foreground_ = map("/home/student/ros2_ws/src/Crop-Guard/image_overlay/images/farmWith3CropRows.pgm");       // Load, filter and rotate PGM image
        
        
        // background_ = cv::imread("/home/andrew/ros2_ws/src/image_overlay/images/farmWithBigPlants.png", cv::IMREAD_UNCHANGED); // PNG image with RGBA channels
        // foreground_ = map("/home/andrew/ros2_ws/src/image_overlay/images/farmWithBigCrops.pgm");
        // foreground_ = map("/home/andrew/ros2_ws/src/image_overlay/images/slam_Map.pgm");

        if (background_.empty() || foreground_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Error loading images!");
            return;
        }
    
        if (background_.size() != foreground_.size()) {
            // RCLCPP_ERROR(this->get_logger(), "Background and foreground sizes do not match. Resizing foreground...");
            cv::resize(foreground_, foreground_, background_.size());
        }

        // Perform the overlay process
        overlayImages();
    }

private:
    cv::Mat background_;
    cv::Mat foreground_;

    // Function to load the PGM image, filter out non-black pixels, and rotate by 90 degrees counter-clockwise
    cv::Mat map(const std::string& input_pgm)
    {
        // Load the PGM image as grayscale
        cv::Mat grayscale = cv::imread(input_pgm, cv::IMREAD_GRAYSCALE);


        // Rotate the image by 90 degrees counter-clockwise (to fix the orientation)
        cv::Mat rotated;
        cv::rotate(grayscale, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);

        // Convert the grayscale (rotated) PGM image to RGBA
        cv::Mat rgba_image;
        cv::cvtColor(rotated, rgba_image, cv::COLOR_GRAY2BGRA);

        // Iterate through the pixels and set alpha channel based on non-black pixels
        for (int y = 0; y < rgba_image.rows; ++y) {
            for (int x = 0; x < rgba_image.cols; ++x) {
                uchar pixel_value = rotated.at<uchar>(y, x);

                // If the pixel is not black, make it fully transparent
                if (pixel_value != 0) {
                    rgba_image.at<cv::Vec4b>(y, x)[3] = 0;  // Set alpha channel to 0 (fully transparent)
                } else {
                    rgba_image.at<cv::Vec4b>(y, x)[3] = 255;  // Set alpha channel to 255 (fully opaque)
                }
            }
        }

        return rgba_image; // Return the processed RGBA image
    }

    void overlayImages()
    {
        // Blend the PNG (background) and PGM (foreground with alpha)
        cv::Mat output = background_.clone();
        for (int y = 0; y < background_.rows; ++y) {
            for (int x = 0; x < background_.cols; ++x) {
                cv::Vec4b bg_pixel = background_.at<cv::Vec4b>(y, x);
                cv::Vec4b fg_pixel = foreground_.at<cv::Vec4b>(y, x);

                // Blend the pixel using alpha channel from the PGM
                float alpha = fg_pixel[3] / 255.0;  // Normalize alpha to range [0, 1]
                for (int c = 0; c < 3; ++c) {  // Blend RGB channels
                    bg_pixel[c] = bg_pixel[c] * (1.0 - alpha) + fg_pixel[c] * alpha;
                }
                // Update the pixel in the output image
                output.at<cv::Vec4b>(y, x) = bg_pixel;
            }
        }

    //         // Scale the output image by 50%
    // cv::Mat scaled_output;
    // cv::resize(output, scaled_output, cv::Size(), 0.5, 0.5);

    //     // Display the composited image
    //     cv::imshow("Composited Image", output);
    //     cv::waitKey(0);

    //     // Optionally, save the resulting image
    //     cv::imwrite("output_composited.png", output);
    // }

        // Scale the output image by 50% (width and height)
    cv::Mat scaled_output;
    cv::resize(output, scaled_output, cv::Size(output.cols / 1.5, output.rows / 1.5));

    // Display the composited image (scaled down)
    cv::imshow("Composited Image (Scaled Down)", scaled_output);
    cv::waitKey(0);

    // Optionally, save the scaled resulting image
    cv::imwrite("output_composited_scaled.png", scaled_output);
}

};

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageOverlay>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

