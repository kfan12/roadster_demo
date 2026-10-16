cmake_minimum_required(VERSION 3.8)
project(roadster_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(av_feature_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
# OSQP / osqp-eigen were built from source into /usr/local (CLAUDE.md gotcha)
set(OsqpEigen_DIR "/usr/local/lib/cmake/OsqpEigen" CACHE PATH "")
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

add_library(roadster_allocator src/allocator.cpp)
target_include_directories(roadster_allocator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(roadster_allocator Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(stunt_mission_node src/stunt_mission_node.cpp)
target_link_libraries(stunt_mission_node roadster_allocator yaml-cpp)
ament_target_dependencies(stunt_mission_node
  rclcpp std_msgs nav_msgs ros_gz_interfaces av_feature_msgs)

install(TARGETS roadster_allocator stunt_mission_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control test/test_control.cpp)
  target_link_libraries(test_control roadster_allocator)
endif()

ament_package()
