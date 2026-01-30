cmake_minimum_required(VERSION 3.8)
project(mtc_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)

# New dependencies for bottle_collision
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- Existing Node (mtc_node) ---
add_executable(mtc_node src/mtc_node.cpp)
target_include_directories(mtc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mtc_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  mtc_node
  "moveit_task_constructor_core"
  "rclcpp"
)

# --- New Node (bottle_collision) ---
add_executable(bottle_collision src/bottle_collision.cpp)
target_compile_features(bottle_collision PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(bottle_collision
  "rclcpp"
  "visualization_msgs"
  "geometry_msgs"
  "moveit_msgs"
  "shape_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
)

# --- Install Targets ---
install(TARGETS 
  mtc_node 
  bottle_collision  # Added new node here
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()