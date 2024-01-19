cmake_minimum_required(VERSION 3.5)
project(pros_arm)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(system_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(serial REQUIRED)

find_package(pros_library REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread program_options)

find_package(trajectory_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(mock_arm src/MockArmNode.cpp)
ament_target_dependencies(mock_arm sensor_msgs trajectory_msgs pros_library)
target_link_libraries(mock_arm pros_library::pros_node)
install(TARGETS
  mock_arm
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(pros_arm src/PROSArmNode.cpp)
ament_target_dependencies(pros_arm pros_library serial sensor_msgs trajectory_msgs )
target_link_libraries(pros_arm pros_library::pros_node)

install(TARGETS
  pros_arm
  DESTINATION lib/${PROJECT_NAME}/
)


add_executable(random_target src/RandomTargetArmNode.cpp)
ament_target_dependencies(random_target pros_library serial sensor_msgs trajectory_msgs )
target_link_libraries(random_target pros_library::pros_node)

install(TARGETS
  random_target
  DESTINATION lib/${PROJECT_NAME}/
)



# add_executable(arm src/PROSArmMain.cpp)
# ament_target_dependencies(arm rclcpp sensor_msgs trajectory_msgs std_msgs serial pros_library)
# target_link_libraries(arm pros_library::pros_node)
# install(TARGETS
#   mock_arm_pub
#   DESTINATION lib/${PROJECT_NAME}
# )
# target_link_libraries(arm pros_library::pros_node)
# include_directories("/workspaces/ros2-log-system/src/serial/build")

# target_link_libraries(mock_arm_pub Serial)
# add_executable(pros_arm src/PROSArmMain.cpp)
# ament_target_dependencies(pros_arm rclcpp sensor_msgs trajectory_msgs pros_library rclcpp_lifecycle std_msgs)
# install(TARGETS
#   pros_arm
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()