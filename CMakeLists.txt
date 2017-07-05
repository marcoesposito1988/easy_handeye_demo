cmake_minimum_required(VERSION 2.8.3)
project(easy_handeye_demo)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
    scripts/noisy_tf_publisher.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
    launch/demo.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
