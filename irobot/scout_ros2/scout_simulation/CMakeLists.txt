cmake_minimum_required(VERSION 3.5)
project(scout_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  worlds
  robots
  models
  rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
