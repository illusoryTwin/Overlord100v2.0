# ***Overlord100***
# Autonomous Mobile Platform

This repository integrates hardware, frontend, and backend modules for the Overlord100 Mobile Platform. The backend module consists of the following components:

1. **Controller**: Includes the high-level controller and various infrastructure packages such as the log collector and mode switcher.
2. **SLAM**: Provides Simultaneous Localization and Mapping functionalities.
3. **Path Planner**: Responsible for determining the optimal path.

![Demo gif](demo.gif)

## Installation & launch
### Docker

Before launching docker, you need to login into the gitlab's docker registry:
```bash
docker loing harbor.stageogip.ru   
```
You need to enter your mail and PAT token.


#### One-liner, simulator 
```docker
docker compose up --build simulator
```

#### Development, simulator
If you want to play around, you could launch the terminal:
```docker
docker compose up --build terminal
```
Inside there, you have to build the project manually:
```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 run overlord100 sim.launch.py
```

You can attach to the docker from other terminals, by running:
```docker
docker compose exec terminal bash
```

#### Launching hardware
TBD

### Local installation
#### Dependencies 
The project software is developed using the ROS2 Humble middleware and relies on the following external dependencies:

1. [Slam Toolbox](https://github.com/SteveMacenski/slam_toolbox)
2. [Nav2](https://github.com/ros-navigation/navigation2)

Additionally, the following ROS2 package is used for hardware components:

3. [Sllidar](https://github.com/Slamtec/sllidar_ros2)

They could be installed at:
```bash
RUN apt-get install -y ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup 
```

#### Launching simulator
```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch overlord100 sim.launch.py
```
#### Launching hardware
TBD
