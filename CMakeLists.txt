cmake_minimum_required(VERSION 2.8.3)
project(pipebot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  motorCmd.msg
  Classification.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES 
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime
)


# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Motor Libraries
add_library(DCMotorLib src/libs/DCMotor.cpp)
add_library(PWMLib src/libs/PWM.cpp)
add_library(GPIOLib src/libs/GPIO.cpp)
add_library(utilLib src/libs/util.cpp)
