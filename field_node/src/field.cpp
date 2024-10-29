#include "field.hpp"

Field::Field() : Node("crop_data_node"), num_row_(0), row_length_(0) {
    // Initialization logic if needed
}

Crop Field::cropCallback(unsigned int row, unsigned int num_in_row) {
    Crop crop = field_.at(row).at(num_in_row);
    return crop;
}

double Field::moistureCallback(unsigned int row, unsigned int num_in_row) {
    double moisture = cropCallback(row, num_in_row).moisture_;
    return moisture;
}



void Field::createField() {
    // for size of num_row_ (n), create n vectors of row_length_. then in another function populate the field.
    field_.resize(num_row_);
    
    for (unsigned int i = 0; i < num_row_; ++i) {
        field_.at(i).resize(row_length_);  // Resize each row to hold row_length_ crops
    }
    RCLCPP_INFO(this->get_logger(), "Field structure created with %d rows and %d crops per row.", num_row_, row_length_);
}

void Field::populateField(const Settings& settings) {
    // Resize the field according to i_ and n_ in settings (rows x crops per row)
    field_.resize(settings.i_);
    for (unsigned int i = 0; i < settings.i_; ++i) {
        field_.at(i).resize(settings.n_);
    }

    // Random number generator for normal distribution between 0.7 and 1.3
    std::random_device rd;  
    std::mt19937 gen(rd());
    std::normal_distribution<> dist(1.0, 0.2);  // Mean = 1.0, StdDev = 0.2

    // Populate based on random or layout
    if (settings.rand_) {
        RCLCPP_INFO(this->get_logger(), "Populating field with random values.");

        // Randomly choose `wacky_crops_` unique crop positions using a set
        std::set<std::pair<unsigned int, unsigned int>> wacky_positions;
        while (wacky_positions.size() < settings.wacky_crops_) {
            unsigned int row = std::rand() % settings.i_;
            unsigned int col = std::rand() % settings.n_;
            wacky_positions.emplace(row, col);  // Only unique positions will be inserted
        }

        // Populate with random values
        for (unsigned int i = 0; i < settings.i_; ++i) {
            for (unsigned int j = 0; j < settings.n_; ++j) {
                Crop& crop = field_.at(i).at(j);

                // Check if this is a wacky crop
                bool is_wacky = wacky_positions.find(std::make_pair(i, j)) != wacky_positions.end();

                if (is_wacky) {
                    crop.moisture_ = std::clamp(dist(gen), 0.7, 1.3);  // Normally distributed random value for moisture
                    crop.nitrate_ = std::clamp(dist(gen), 0.7, 1.3);  // Normally distributed random value for nitrate
                } else {
                    crop.moisture_ = 1.0;  // Default value for normal crops
                    crop.nitrate_ = 1.0;
                }
            }
        }
    } else {
        // Switch case for predefined layouts
        switch (settings.layout_) {
            case 1:
                RCLCPP_INFO(this->get_logger(), "Populating field using layout 1.");
                for (unsigned int i = 0; i < settings.i_; ++i) {
                    for (unsigned int j = 0; j < settings.n_; ++j) {
                        field_.at(i).at(j).moisture_ = 1.0;  // Example for layout 1
                        field_.at(i).at(j).nitrate_ = 1.0;
                    }
                }
                break;
            
            case 2:
                RCLCPP_INFO(this->get_logger(), "Populating field using layout 2.");
                // Add custom logic for layout 2
                break;

            // Add more layouts as needed

            default:
                RCLCPP_WARN(this->get_logger(), "Unknown layout, using default values.");
                for (unsigned int i = 0; i < settings.i_; ++i) {
                    for (unsigned int j = 0; j < settings.n_; ++j) {
                        field_.at(i).at(j).moisture_ = 1.0;
                        field_.at(i).at(j).nitrate_ = 1.0;
                    }
                }
                break;
        }
    }

    RCLCPP_INFO(this->get_logger(), "Field populated.");
}

// void Field::setSettings(unsigned int num_rows, unsigned int length_rows, 
//         bool rand, unsigned int layout, unsigned int wacky_crops){
//             settings_ = (num_rows, length_rows, rand, layout, wacky_crops); 
//         };

std::vector<std::vector<Crop>> Field::fieldCallback() {
    return field_;
}

int main(int argc, char *argv[]) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create the node
    auto node = std::make_shared<Field>();

    // Spin the node to keep it alive
    rclcpp::spin(node);

    // Shutdown ROS 2 when finished
    rclcpp::shutdown();
    return 0;
}