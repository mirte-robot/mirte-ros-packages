# mirte-ros-packages

This package provides the ROS2 pcakages for the [MIRTE robot](https://mirte.org).
Please read the [MIRTE documentation](https://docs.mirte.org/develop/doc/mirte_os/supported_hardware.html) 
for further documentation of this repository.

## Install

```sh
git submodule update --init --recursive
rosdep install -y --from-paths src/ --ignore-src --rosdistro humble
colcon build --symlink-install
```

The mirte-telemetrix-cpp package needs a connected microcontroller with
the [MIRTE telemetrix server installed](https://docs.mirte.org/develop/doc/mirte_os/install_mirte_software.html#install-mcu-software).

## Checks

To contribute to this repository the code needs to pass the python and c++
style checks.

- Python stylechecks:


  To check this locally before you commit/push:
  ```sh
  pip install black
  black --check **/**.py
  # Fix by using
  black **/**.py
  ```

- C++ stylechecks:

  To check this locally before you commit/push:
  ```sh
  sudo apt install clang-format
  clang-format --dry-run --Werror ./**/**.cpp -style=llvm
  # Fix by using
  clang-format --Werror ./**/**.cpp -style=llvm -i
  ```

## License

This work is licensed under a Apache-2.0 OSS license.
