cmake_minimum_required(VERSION 3.5)

project(eagle_mpc_2_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(eagle_mpc_2_interfaces REQUIRED)
find_package(pinocchio REQUIRED)
find_package(crocoddyl REQUIRED)
find_package(eagle_mpc)

set(EAGLE_MPC_2_TOOLS_SRC src/state_pub_sub.cpp src/controller_base.cpp)
set(DECENTRALIZED_CONTROL_SRC src/decentralized_controller.cpp)
set(MPC_CONTROL_SRC src/mpc_runner.cpp)
set(MOTOR_TEST_SRC src/motor_test.cpp)
set(MPC_TEST src/mpc_runner_single.cpp)

#################
# Setup targets #
#################
include_directories(include 
  ${EIGEN3_INCLUDE_DIR} 
  ${PINOCCHIO_INCLUDE_DIRS}
  ${CROCODDYL_INCLUDE_DIRS}
  ${eagle_mpc_INCLUDE_DIRS} 
)

add_library(eagle_mpc_2_tools ${EAGLE_MPC_2_TOOLS_SRC})
ament_target_dependencies(eagle_mpc_2_tools rclcpp px4_msgs px4_ros_com eagle_mpc_2_interfaces)

add_executable(decentralized_control ${DECENTRALIZED_CONTROL_SRC}) 
ament_target_dependencies(decentralized_control rclcpp px4_msgs px4_ros_com eagle_mpc_2_interfaces)
target_link_libraries(decentralized_control eagle_mpc_2_tools)

add_executable(mpc_runner ${MPC_CONTROL_SRC})
ament_target_dependencies(mpc_runner rclcpp px4_msgs px4_ros_com eagle_mpc_2_interfaces)
target_link_libraries(mpc_runner 
  eagle_mpc_2_tools 
  ${PINOCCHIO_LIBRARIES}
  ${CROCODDYL_LIBRARIES} 
  ${EAGLE_MPC_LIBRARIES} 
)

add_executable(motor_test ${MOTOR_TEST_SRC}) 
ament_target_dependencies(motor_test rclcpp px4_msgs px4_ros_com eagle_mpc_2_interfaces)
target_link_libraries(motor_test eagle_mpc_2_tools)

add_executable(mpc_test ${MPC_TEST}) 
ament_target_dependencies(mpc_test rclcpp px4_msgs px4_ros_com eagle_mpc_2_interfaces)
target_link_libraries(mpc_test eagle_mpc_2_tools)

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

install(TARGETS
  decentralized_control
  motor_test
  mpc_runner
  mpc_test
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
