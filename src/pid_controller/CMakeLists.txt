cmake_minimum_required(VERSION 3.8)
project(pid_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =======================
# Dependencies
# =======================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# =======================
# Build section
# =======================

# Include headers from include/
include_directories(include)

# Build PID as a library (so it can be reused)
add_library(pid src/pid.cpp)
ament_target_dependencies(pid rclcpp std_msgs)

# Build PID node
add_executable(pid_node src/pid_node.cpp)
ament_target_dependencies(pid_node rclcpp std_msgs)
target_link_libraries(pid_node pid)

# Build Plant node
add_executable(plant_node src/plant_sim_node.cpp)
ament_target_dependencies(plant_node rclcpp std_msgs)

# =======================
# Install section
# =======================
install(TARGETS
  pid
  pid_node
  plant_node
  DESTINATION lib/${PROJECT_NAME})

# =======================
# Testing
# =======================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
