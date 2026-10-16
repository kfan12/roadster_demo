cmake_minimum_required(VERSION 3.8)
project(roadster_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch worlds maps models DESTINATION share/${PROJECT_NAME})

ament_package()
