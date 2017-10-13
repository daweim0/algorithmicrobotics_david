# Algorithmic Robotics #

Common code for class. Currently many of the packages are from the duckietown/Software github repository or built off of said packages.

### Contents ###
* ar_tags/
    * src/**tag_detector_node.py** - publishes AR tag detections (IDs, relative positions)
* dagu_car/
    * launch/**dagu_car.launch** - starts the dagu_car nodes and remaps topics
    * script/**drive_straight_test.py** - run this to test the wheel calibration
    * src/**kinematics_node.py** - converts car velocities to wheel speeds
    * src/**wheels_driver_node.py** - sets wheel speeds based on received commands
* duckietown/
    * config/dagu_car/kinematics/**pi.yaml** - wheel calibration
    * config/ground_projection/**pi.yaml** - extrinsic camera calibration
    * config/pi_camera/**pi.yaml** - intrinsic camera calibration
    * launch/ - launch files that start nodes across multiple packages
* duckietown_msgs/
    * msg/ - message definitions used across duckietown
* ground_projection/
    * scripts/**test_projection_auto.py** - run this to test extrinsic camera calibration
* intersection_control/
    * config/**maneuvers.yaml** - open-loop turn descriptions
    * config/**map.yaml** - graph representation of roads
    * src/**stop_line_filter.py** - publishes stop-line readings
    * src/**supervisor_node.py** - manages intersections (and control mode switching)
* keyboard_control/
    * src/**keyboard_control_node.py** - publishes car velocities based on key presses
* lane_control/
    * src/**lane_controller_node.py** - publishes car velocities based on lane pose estimates
* lane_filter/
    * src/**lane_filter_node.py** - publishes lane pose estimates
* localization/
    * config/**landmarks.yaml** - AR tag IDs and locations
    * src/**localization_node.py** - estimates car pose based on AR tags and odometry (lab 10)
* pi_camera/
    * launch/**camera.launch** - starts the camera nodes and remaps topics
    * src/**cam_info_node.py** - publishes camera calibration info each time an image is received
    * src/**camera_node.py** - publishes compressed camera images

### Environment ###

* Raspberry Pi 3, Ubuntu Mate 14.04, Python 2.7.12, OpenCV 2.4.9, ROS Kinetic
