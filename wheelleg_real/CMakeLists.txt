cmake_minimum_required(VERSION 3.0.2)
project(wheelleg_real)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  wheelleg_control
)

# Declare the message files to be built
# add_message_files(FILES
#   # JointData.msg
#   # MotorData.msg
# )

# Actually generate the language-specific message and service files
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Declare that this catkin package's runtime dependencies
catkin_package(
  CATKIN_DEPENDS message_runtime
)




add_compile_options(-lpthread)

include_directories(${catkin_INCLUDE_DIRS} src/socketCan2/inc)

set(SRCS 
    src/socketCan2/src/CanManager.cpp
    src/socketCan2/src/Math.cpp
    src/socketCan2/src/M2006.cpp
    src/socketCan2/src/Pid.cpp
    src/socketCan2/src/M3508.cpp
    )

# add_executable(chassisController ${SRCS} src/ChassisController.cpp )
# target_link_libraries(chassisController ${catkin_LIBRARIES} pthread)

add_executable(motorController ${SRCS} src/MotorController.cpp )
target_link_libraries(motorController ${catkin_LIBRARIES} pthread)
