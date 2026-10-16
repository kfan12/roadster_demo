cmake_minimum_required(VERSION 3.8)
project(roadster_gz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# the one force-writing Gazebo system: tank + 10 nozzles + suction fan
add_library(roadster_actuation_system SHARED src/roadster_actuation_system.cpp)
target_include_directories(roadster_actuation_system PRIVATE include)
target_link_libraries(roadster_actuation_system
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::register
  ignition-transport11::ignition-transport11
  ignition-msgs8::ignition-msgs8
  yaml-cpp
  nlohmann_json::nlohmann_json)

install(TARGETS roadster_actuation_system LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# Gazebo finds system plugins through IGN_GAZEBO_SYSTEM_PLUGIN_PATH
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_models test/test_models.cpp)
  target_include_directories(test_models PRIVATE include)
endif()

ament_export_include_directories(include)
ament_package()
