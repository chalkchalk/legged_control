cmake_minimum_required(VERSION 3.0.2)
project(doggy_tele)

# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_STANDARD 14)


find_package(catkin REQUIRED COMPONENTS
  doggy_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  ocs2_legged_robot_ros
)

catkin_package( #MUST add this to build the executable file in /lib for rosrun!!!!!!
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp std_msgs sensor_msgs ocs2_legged_robot_ros
        #  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(joy_to_cmd
  src/joy_to_cmd.cpp
  )

target_link_libraries(joy_to_cmd
        ${catkin_LIBRARIES}
        )

add_executable(cmd_to_ocs2
      src/cmd_to_ocs2.cpp
      )

target_link_libraries(cmd_to_ocs2
      ${catkin_LIBRARIES}
      )
