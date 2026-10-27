# RoboRacer simulation

ROS 2 Jazzy and Gazebo Harmonic simulation package.
Use your existing container.

## Install

Inside the container, clone into the workspace's `src` directory:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Mac-Adam/Autka.git roboracer_sim
```

In the docker, import it and install dependencies with:

```bash
bash ~/ros2_ws/src/.devcontainer/setup.sh
```

## Build and run

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --merge-install --symlink-install --packages-up-to roboracer_sim
source install/setup.bash
ros2 launch roboracer_sim simulation.launch.py
```

The launch starts Gazebo, RViz, and the LiDAR driving controller.
Press Ctrl+C to stop.