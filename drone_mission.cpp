#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <gnc_functions.hpp>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

// Global variables for obstacle and human detection
bool avoid_obstacle = false;
bool human_detected = false;
float altitude = 0.0;
std::vector<gnc_api_waypoint> waypointList;
int counter = 0;
float newheight;
float lastx = 0;
float lasty = 0;
float dist;
cv::Mat current_frame;

// Structure to hold detection information
struct Detection {
    std::string class_name;
    float x;
    float y;
    std::string timestamp;
    float probability;
};

// List to accumulate detections
std::vector<Detection> detections;

// Function to calculate the distance between two points
double calculateDistance(float x1, float y1, float x2, float y2) {
    return abs(sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)));
}

// Function to hold position for a specified duration
void hold_position() {
    ros::Duration(5).sleep(); // Adjust as needed
    ros::spinOnce();
    
    // Reset flag after holding position
    set_destination(waypointList[counter-1].x, waypointList[counter-1].y, get_current_location().z, get_current_heading());
}

// Function to get the current timestamp as a string
std::string get_current_time() {
    std::time_t t = std::time(nullptr);
    char mbstr[100];
    if (std::strftime(mbstr, sizeof(mbstr), "%Y-%m-%d %H:%M:%S", std::localtime(&t))) {
        return std::string(mbstr);
    }
    return "";
}

// Function to save the current frame as an image
void save_image(const std::string& directory) {
    if (!current_frame.empty()) {
        std::string timestamp = get_current_time();
        std::replace(timestamp.begin(), timestamp.end(), ' ', '_');
        std::replace(timestamp.begin(), timestamp.end(), ':', '-');
        std::string filename = directory + "/mission_images" + "/person_detected_" + timestamp + ".png";
        cv::imwrite(filename, current_frame);
        ROS_INFO("Image saved: %s", filename.c_str());
    } else {
        ROS_WARN("No frame to save.");
    }
}

// Callback function for detection
void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg) {
    if (!human_detected) {
        for (int i = 0; i < msg->bounding_boxes.size(); i++) {
            if (msg->bounding_boxes[i].Class == "person" && msg->bounding_boxes[i].probability > 0.65) {
                geometry_msgs::Point point = get_current_location();
                dist = calculateDistance(lastx, lasty, point.x, point.y);
                //ROS_INFO("distance %f", dist);
                if (dist < 4.0) {
                    return;
                }

                // Stop the drone by setting the destination to the current position
                set_destination(point.x, point.y, point.z, get_current_heading());

                lastx = point.x;
                lasty = point.y;

                ROS_INFO("%s detected", msg->bounding_boxes[i].Class.c_str());
                ROS_INFO("location: %f, %f", lastx, lasty);
                human_detected = true;  // Set the human_detected flag

                ROS_INFO("Probability: %f", msg->bounding_boxes[i].probability);
                save_image("/home/hassaan/catkin_ws/src/iq_gnc/src");  // Save the image

                // Accumulate detection information
                Detection detection;
                detection.class_name = msg->bounding_boxes[i].Class;
                detection.x = lastx;
                detection.y = lasty;
                detection.timestamp = get_current_time();
                detection.probability = msg->bounding_boxes[i].probability;
                detections.push_back(detection);
                return;
            }
        }
    }
}

// Callback function for scan data
void scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg) {
    sensor_msgs::LaserScan current_2D_scan = *msg;
    avoid_obstacle = false;
    
    for (int i = 1; i < current_2D_scan.ranges.size(); i++) {
        float d0 = 5;
        if (current_2D_scan.ranges[i] < d0 && current_2D_scan.ranges[i] > 0.35) {
            avoid_obstacle = true;
            ROS_INFO("Obstacle detected");
            return;
        }
    }
}

// Function to increase altitude until clear of obstacles
void increase_altitude_until_clear() {
    newheight = waypointList[counter-1].z;
    geometry_msgs::Point loc = get_current_location();

    while (avoid_obstacle) {
        newheight = newheight + 1.0;
        set_destination(loc.x, loc.y, newheight, get_current_heading());
        ros::Duration(2).sleep(); // Adjust as needed
        ros::spinOnce();
    }
    set_destination(waypointList[counter-1].x, waypointList[counter-1].y, newheight, get_current_heading());
}

// Function to read waypoints from a CSV file
void read_waypoints_from_csv(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open the CSV file: %s", filename.c_str());
        return;
    }

    std::string line;
    bool is_first_line = true;
    float initial_altitude = 0;

    while (std::getline(file, line)) {
        // Skip empty lines
        if (line.empty()) {
            continue;
        }

        std::istringstream ss(line);
        gnc_api_waypoint waypoint;
        std::string token;

        try {
            std::getline(ss, token, ',');
            waypoint.x = std::stof(token);
            std::getline(ss, token, ',');
            waypoint.y = std::stof(token);
            std::getline(ss, token, ',');
            waypoint.psi = std::stof(token);

            if (is_first_line) {
                if (std::getline(ss, token, ',')) {
                    initial_altitude = std::stof(token);  // Read altitude from the first line
                } else {
                    ROS_ERROR("Initial altitude not found in the first line of CSV");
                    return;
                }
                waypoint.z = initial_altitude;
                altitude = initial_altitude; // Set global altitude
                is_first_line = false;
            } else {
                waypoint.z = initial_altitude;  // Use the initial altitude for subsequent waypoints
            }

            waypointList.push_back(waypoint);
            ROS_INFO("Read waypoint - x: %f, y: %f, psi: %f, altitude: %f", waypoint.x, waypoint.y, waypoint.psi, waypoint.z);
        } catch (const std::invalid_argument& e) {
            ROS_ERROR("Invalid data in CSV file: %s", e.what());
            continue;
        } catch (const std::out_of_range& e) {
            ROS_ERROR("Out of range error in CSV file: %s", e.what());
            continue;
        }
    }

    file.close();
}

// Function to write detections to a CSV file
void write_detections_to_csv(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open the CSV file for writing detections: %s", filename.c_str());
        return;
    }

    // Write the header
    file << "Class, x, y, Timestamp, Probability" << std::endl;

    // Write each detection
    for (const auto& detection : detections) {
        file << detection.class_name << ", "
             << detection.x << ", "
             << detection.y << ", "
             << detection.timestamp << ", "
             << detection.probability << std::endl;
    }

    file.close();
}

// Callback function for image data
void image_cb(const sensor_msgs::ImageConstPtr& msg) {
    try {
        current_frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
        //ROS_INFO("Image received");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "autonomous_drone_navigation");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    // Subscribe to sensors and detection
    ros::Subscriber collision_sub = n.subscribe<sensor_msgs::LaserScan>("/spur/laser/scan", 5, scan_cb);
    ros::Subscriber detection_sub = n.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 20, detection_cb);
    image_transport::Subscriber image_sub = it.subscribe("/webcam/image_raw", 1, image_cb);

    // Initialize necessary ROS communications
    init_publisher_subscriber(n);
    wait4connect();
    wait4start();
    initialize_local_frame();
    set_speed(1);

    // Specify the full path to the CSV file
    std::string csv_file_path = "/home/hassaan/catkin_ws/src/iq_gnc/src/waypoints.csv";  // Update this path to the actual path of your CSV file
    read_waypoints_from_csv(csv_file_path);

    if (altitude == 0.0) {
        ROS_ERROR("Altitude is not set. Please check the CSV file.");
        return -1;
    }

    // Print waypoints for debugging
    for (const auto& wp : waypointList) {
        ROS_INFO("Waypoint - x: %f, y: %f, psi: %f, altitude: %f", wp.x, wp.y, wp.psi, wp.z);
    }

    takeoff(altitude);

    // Use AsyncSpinner to handle callbacks in parallel
    ros::AsyncSpinner spinner(3); // Specify the number of threads to use
    spinner.start();

    // Control loop
    ros::Rate rate(2.0);
    while (ros::ok()) {
        if (avoid_obstacle) {
            increase_altitude_until_clear();
        } else if (check_waypoint_reached(0.3) == 1) {
            if (counter < waypointList.size()) {
                if (human_detected) {
                    ROS_INFO("Location reached, holding position");
                    hold_position();
                    human_detected = false;
                } else {
                    set_destination(waypointList[counter].x, waypointList[counter].y, waypointList[counter].z, waypointList[counter].psi);
                    counter++;
                }
            } else {
                land();
                break;
            }
        }
        rate.sleep();
    }

    // Write the accumulated detections to a CSV file
    std::string detection_csv_file_path = "/home/hassaan/catkin_ws/src/iq_gnc/src/detection_log.csv";  // Update this path to where you want to store the detection log
    write_detections_to_csv(detection_csv_file_path);

    return 0;
}
