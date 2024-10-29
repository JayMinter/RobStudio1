#ifndef FIELD_HPP
#define FIELD_HPP

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "field_structs.hpp"
#include <random>

/**
 * @class Field
 * @brief A class for managing crop data in a ROS2 node.
 */
class Field : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the Field node.
     */
    Field();

private:
    // Setters and getters.

    Crop cropCallback(unsigned int row, unsigned int num_in_row);
    double moistureCallback(unsigned int row, unsigned int num_in_row);



    void createField();
    void populateField(const Settings& settings);
    std::vector<std::vector<Crop>> fieldCallback();

    // Sets settings based on gui outputs
    void setSettings(unsigned int num_rows, unsigned int length_rows, 
        bool rand, unsigned int layout, unsigned int wacky_crops);

    //void rowPushback();
    //void cropPushback();double moistureCallback()
    //void dataPushback();

    // Structure Plan: vector of rows -> vector of crops -> vector of datapoints
    std::vector<std::vector<Crop>> field_;   // Field is a vector of rows, each row is a vector of Crop objects
    unsigned int num_row_;
    unsigned int row_length_;
    Settings settings_;
};

#endif  // FIELD_HPP
