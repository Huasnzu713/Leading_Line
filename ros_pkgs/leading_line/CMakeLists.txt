cmake_minimum_required(VERSION 3.0.2)
project(leading_line)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    rospy
    sensor_msgs
    std_msgs
    cv_bridge
)

#############
## Install ##
#############

# 所有 Python 节点装到 ${CATKIN_PACKAGE_BIN_DESTINATION}
catkin_install_python(PROGRAMS
  scripts/node.py
  scripts/pipeline_launcher.py
  scripts/xcar/xcar_ros.py
  scripts/xcar/xcar_protocol.py
  scripts/xcar/mbot_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# launch / config 文件装到 share/leading_line/
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
