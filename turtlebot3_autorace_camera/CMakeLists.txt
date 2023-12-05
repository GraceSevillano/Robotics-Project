cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_autorace_camera)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/ImageProjectionParams.cfg
  cfg/ImageCompensationParams.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure
)

catkin_install_python(PROGRAMS
  nodes/image_compensation
  nodes/image_projection
 	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY calibration cfg launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
