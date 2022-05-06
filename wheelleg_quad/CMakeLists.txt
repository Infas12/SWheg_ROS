cmake_minimum_required(VERSION 3.0.2)
project(wheelleg_quad)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

# Declare the message files to be built
add_message_files(FILES
  JointData.msg
)

# Actually generate the language-specific message and service files
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Declare that this catkin package's runtime dependencies
catkin_package(
  CATKIN_DEPENDS message_runtime
)


catkin_install_python(PROGRAMS scripts/test.py
scripts/test0223.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/linearController.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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

add_executable(chassisController ${SRCS} src/ChassisController.cpp )
target_link_libraries(chassisController ${catkin_LIBRARIES} pthread)

add_executable(motorController ${SRCS} src/MotorController.cpp )
target_link_libraries(motorController ${catkin_LIBRARIES} pthread)
