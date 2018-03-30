cmake_minimum_required(VERSION 2.8.3)
project(pirosbot)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation message_runtime)

## Declare ROS messages and services
add_message_files(
	FILES 
	DC_Control.msg
	CAM_Control.msg
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()


set(MY_INCLUDE_DIRS 
	"include/" 
	"include/rpi_motor_driver/" 
#	"include/rpi_motor_driver/PIGPIO/"
	"/usr/local/include/"
	)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${MY_INCLUDE_DIRS})

link_directories("/usr/local/lib/")


add_executable(driver src/driver.cpp src/rpi_motor_driver/rpi_driver.cpp)
target_link_libraries(driver ${catkin_LIBRARIES} libpigpio.so)
add_dependencies(driver pirosbot_generate_messages_cpp)

add_executable(driverd_motor src/driverd_motor.cpp src/rpi_motor_driver/rpi_driverd_mot.cpp)
target_link_libraries(driverd_motor ${catkin_LIBRARIES} libpigpiod_if2.so)
add_dependencies(driverd_motor pirosbot_generate_messages_cpp)

add_executable(driverd_camera src/driverd_camera.cpp src/rpi_motor_driver/rpi_driverd_cam.cpp)
target_link_libraries(driverd_camera ${catkin_LIBRARIES} libpigpiod_if2.so)
add_dependencies(driverd_camera pirosbot_generate_messages_cpp)

add_executable(key_controller src/key_controller.cpp)
target_link_libraries(key_controller ${catkin_LIBRARIES})
add_dependencies(key_controller pirosbot_generate_messages_cpp)

# add_executable(joy_controller src/joy_controller.cpp)
# target_link_libraries(joy_controller ${catkin_LIBRARIES})
# add_dependencies(joy_controller pirosbot_generate_messages_cpp)

