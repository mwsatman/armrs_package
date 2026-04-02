# armrs_package
Code repository for cooperative control of multi-robot system

To be used in Aerial Robotics and Multi-Robot System Course

Each of the file describes the components for easier separation of algorithm in python, ROS wrapper, and real-robot implementation. Thus, ease the process of transition from proof-of-concept to verification

- yaml files: contain simulation-specific setups and scenario-specific parameters
- yaml_loader: parse the yaml files and store them into the designated class for easier access and categorization
- simulator.py: main calculation to model the movement of the robots based on the control input
- visualizer.py: basic encapsulation of visualizing the movement and range-sensor measurement of the robots
- main_controller.py: contain the encapsulation of estimation and controller calculation for each robot


## Setup in ROS2 HUMBLE

- install dependency `pip3 install pyyaml`
- clone this folder inside `~/ros2_ws/src`
- `colcon build --symlink-install`
- `source install/setup.bash`

- on ROS2_sim_launch, remember to correct the absolute path of the yaml file.
- `ros2 launch mrs_cbf_journal ROS2_sim_launch.py`

