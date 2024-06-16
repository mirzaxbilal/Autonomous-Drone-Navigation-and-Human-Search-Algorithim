Autonomous Drone Navigation and Human Detection Simulation
Overview
This project features an autonomous drone simulation in Gazebo, designed to follow a series of user-defined waypoints, detect human presence using YOLO (You Only Look Once), capture images upon detection, and log the coordinates of detected humans in a CSV file. The simulation leverages ROS Noetic for communication and control, and uses LiDAR for obstacle detection and avoidance.

Features
Waypoint Navigation: The drone follows a sequence of waypoints defined in a CSV file.
Obstacle Detection and Avoidance: Uses LiDAR to detect obstacles and adjust altitude to avoid collisions.
Human Detection: Utilizes YOLO for detecting humans in the drone's path.
Image Capture: Captures and saves images when a human is detected.
Logging: Records the coordinates and details of detected humans in a CSV file.
Dependencies
ROS Noetic: Robot Operating System for communication and control.
Gazebo: 3D robotics simulator.
YOLO: Real-time object detection system.
OpenCV: Library for image processing.
cv_bridge: ROS package to convert between ROS and OpenCV images.
darknet_ros: ROS wrapper for YOLO object detection.
sensor_msgs: ROS messages for sensor data.
geometry_msgs: ROS messages for geometric primitives.
gnc_functions: Custom GNC (Guidance, Navigation, and Control) functions for drone control.

Functionality
Waypoint Navigation
The drone reads waypoints from a CSV file and follows them sequentially. If an obstacle is detected, the drone adjusts its altitude to avoid the obstacle.

Human Detection
The drone uses YOLO for human detection. Upon detecting a human with a probability above 65%, the drone captures an image and logs the coordinates along with detection details in a CSV file.

Obstacle Detection
LiDAR is used to detect obstacles. If an obstacle is within a predefined distance, the drone will increase its altitude until the path is clear.

Code Explanation
Main Components
Global Variables: Variables to track state, such as avoid_obstacle, human_detected, and waypointList.
Detection Callback: Processes YOLO detection messages, captures images, and logs detection data.
Scan Callback: Processes LiDAR scan data to detect obstacles.
Waypoint Navigation: Reads waypoints from a CSV file and navigates the drone.
Image Capture and Logging: Captures images upon human detection and logs detection details to a CSV file.

Key Functions
calculateDistance(): Calculates distance between two points.
hold_position(): Holds the drone's position for a specified duration.
get_current_time(): Returns the current timestamp as a string.
save_image(): Saves the current frame as an image.
detection_cb(): Callback for processing detection messages.
scan_cb(): Callback for processing scan data.
increase_altitude_until_clear(): Increases altitude to avoid obstacles.
read_waypoints_from_csv(): Reads waypoints from a CSV file.
write_detections_to_csv(): Writes detection data to a CSV file.
image_cb(): Callback for processing image data.
