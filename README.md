# Usage

Check the [MIRTE documentation](https://docs.mirte.org/) for usage and ROS API.

# Build

To build all the MIRTE ROS2 packages, you can build them:

```sh
cd ~/ros2_ws/src
git clone https://gitub.com/mirte-robot/mirte-ros-packages
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

# Test C++ and python code style

The following checks are done in the github action pipeline. You can also
run (and fix) these before pushing.

To check the C++ and Python code style run:
```sh
pip install black
black --check **/**.py
# Fix by using
black **/**.py

sudo apt install clang-format # preferably version 14, 10 should be fine
clang-format --dry-run --Werror ./**/**.cpp -style=llvm
# Fix by using
clang-format --Werror ./**/**.cpp -style=llvm -i
```
