cmake_minimum_required(VERSION 3.0.2)
project(jetson_camera)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
  message_generation
  nav_msgs
)

add_message_files(
  FILES
  ImagePair.msg
  QRdata.msg
  FACEdata.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime cv_bridge rospy sensor_msgs std_msgs
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install Python scripts
catkin_install_python(PROGRAMS
  src/camera_publisher_node.py
  #src/camera_subscriber_node.py
  #src/image_processing_node.py
  #src/QRParkingNode.py
  #src/face_rec.py
  #src/pose_estimation.py
  src/taskManager.py
  src/routing.py
  #src/simplePark.py
  #src/barcode.py
  src/qr_navigation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)