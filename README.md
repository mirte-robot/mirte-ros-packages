```bash
echo "deb [trusted=yes] https://github.com/mirte-robot/mirte-ros-packages/raw/ros_mirte_humble_jammy_amd64/ ./" | sudo tee /etc/apt/sources.list.d/mirte-robot_mirte-ros-packages.list
echo "yaml https://github.com/mirte-robot/mirte-ros-packages/raw/ros_mirte_humble_jammy_amd64/local.yaml humble" | sudo tee /etc/ros/rosdep/sources.list.d/1-mirte-robot_mirte-ros-packages.list
```
