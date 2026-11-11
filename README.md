# RoboRacer simulation

ROS 2 Jazzy and Gazebo Harmonic simulation package.
Use your existing container.

## 2D mapping

The simulation launch starts Cartographer by default, using the LiDAR `/scan`
and simulated `/odom` topics. RViz displays the live occupancy grid in the
`map` frame. To run without SLAM, use `slam:=false rviz_fixed_frame:=odom`.

Drive around the environment to build coverage. While the simulation is still
running, save the completed map from this package directory:

```bash
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/track"
```

The command writes `maps/track.yaml` and `maps/track.pgm`. Both are required
when reusing the map; cells not observed during the drive remain unknown.

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

The launch starts Gazebo, RViz, LiDAR path estimator and pure pursuit driving controller.
Press Ctrl+C to stop.

## Open container in local terminal

Expose GUI
```bash
xhost +local:docker
```
Build container
```bash
docker compose -f .devcontainer/compose.yaml up -d --build
```

Enter working terminal
```bash
docker compose -f .devcontainer/compose.yaml exec workspace bash
```

Cleanup
```bash
docker compose -f .devcontainer/compose.yaml down
xhost -local:docker
```