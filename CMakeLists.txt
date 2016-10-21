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
   sensorData.msg
   sensorDetection.msg
   accelData.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES 
  bodyMotorsSrv.srv
  neckMotorsSrv.srv
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
# add_library(Adafruit BBIO lib)

# Pipebot Nodes (C++)
add_executable(conductor src/conductor.cpp)
target_link_libraries(conductor ${catkin_LIBRARIES})
add_dependencies(conductor pipebot_generate_messages_cpp)

add_executable(bodyMotors src/bodyMotors.cpp)
target_link_libraries(bodyMotors ${catkin_LIBRARIES} DCMotorLib PWMLib GPIOLib utilLib)
add_dependencies(bodyMotors pipebot_generate_messages_cpp)

add_executable(neckMotors src/neckMotors.cpp)
target_link_libraries(neckMotors ${catkin_LIBRARIES} PWMLib utilLib)
add_dependencies(neckMotors pipebot_generate_messages_cpp)
