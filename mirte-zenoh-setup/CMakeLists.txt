cmake_minimum_required(VERSION 3.8)
project(mirte_zenoh_setup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
        find_package(ament_cmake_core QUIET REQUIRED)
    set(mirte_zenoh_setup_HOME ${CATKIN_PACKAGE_SHARE_DESTINATION})

ament_environment_hooks(env_hook/mirte_zenoh_setup.sh.in)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

find_package(ament_cmake REQUIRED)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
