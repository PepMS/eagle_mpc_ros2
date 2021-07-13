cmake_minimum_required(VERSION 3.5)

project(px4_mpc)

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
find_package(eagle_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(PX4_MPC_OFFBOARD_SRC src/offboard_control.cpp)

# SET(MULTICOPTER_MPC_INCLUDE_DIRS /usr/local/include/multicopter_mpc/)
# SET(MULTICOPTER_MPC_LIB /usr/local/lib/libmulticopter_mpc.so)

file(GLOB_RECURSE PX4_MPC_SOURCES *.cpp)

#################
# Setup targets #
#################
include_directories(include ${EIGEN3_INCLUDE_DIR})

add_executable(px4_offboard_control ${PX4_MPC_OFFBOARD_SRC}) 
ament_target_dependencies(px4_offboard_control rclcpp px4_msgs px4_ros_com eagle_interfaces)
# target_link_libraries(px4_controller ${MULTICOPTER_MPC_LIB})

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
px4_offboard_control
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
