cmake_minimum_required(VERSION 2.8.3)
project(beacon_simple_nav)

find_package(catkin REQUIRED COMPONENTS message_generation rospy)



catkin_python_setup()
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
catkin_package()