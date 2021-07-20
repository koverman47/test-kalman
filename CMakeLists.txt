cmake_minimum_required(VERSION 3.0.2)
project(test_kalman)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs message_generation)

add_message_files(
    FILES
    State.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)