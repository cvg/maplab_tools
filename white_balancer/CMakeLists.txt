cmake_minimum_required(VERSION 3.1)
project(white_balancer)

add_definitions(-Wall)

find_package(catkin REQUIRED
  COMPONENTS
)

catkin_package(
  CATKIN_DEPENDS
)

install(DIRECTORY bin config launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
