cmake_minimum_required(VERSION 3.0.2)
project(easy_handeye_demo)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
    scripts/tracking_simulator.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
    launch/calibrate.launch
    launch/check_calibration.launch
    launch/iiwa_stack_config.rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
