#ifndef CROPDATA_HPP
#define CROPDATA_HPP

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "field_structs.hpp"

/**
 * @class CropData
 * @brief A class for managing crop data in a ROS2 node.
 */
class CropData : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the CropData node.
     */
    CropData();

private:
    // Setters and getters.

    Crop cropCallback();
    double moistureCallback();



    void createField();
    void populateField(const Settings& settings);
    std::vector<std::vector<Crop>> fieldCallback();

    // Sets settings based on gui outputs
    void setSettings(unsigned int num_rows, unsigned int length_rows, 
        bool rand, unsigned int layout, unsigned int wacky_crops);

    void rowPushback();
    void cropPushback();
    void dataPushback();

    // Structure Plan: vector of rows -> vector of crops -> vector of datapoints
    std::vector<std::vector<Crop>> field_;   // Field is a vector of rows, each row is a vector of Crop objects
    unsigned int num_row_;
    unsigned int row_length_;
    Settings settings_;
};

#endif  // CROPDATA_HPP
