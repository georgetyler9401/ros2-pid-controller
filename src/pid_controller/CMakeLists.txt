cmake_minimum_required(VERSION 3.8)
project(pid_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# =======================
# Build section
# =======================

# include headers
include_directories(include)

# build the PID node executable (linking pid.cpp and pid_node.cpp)
add_executable(pid_node src/pid_node.cpp src/pid.cpp)
ament_target_dependencies(pid_node rclcpp std_msgs)

# =======================
# Install section
# =======================

install(TARGETS
  pid_node
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
