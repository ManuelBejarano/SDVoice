cmake_minimum_required(VERSION 2.8.3)
project(rqt_sdvoice)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  rqt_gui
  rqt_gui_py
  sensor_msgs
  std_msgs
)
catkin_python_setup()
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/rqt_sdvoice.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)






