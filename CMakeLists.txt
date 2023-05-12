cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

project(daheng_galaxy)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  camera_info_manager
)

set (DYNAMIC_RECONFIGURE_PATH 
${CMAKE_CURRENT_SOURCE_DIR}/cfg/camera.cfg )

generate_dynamic_reconfigure_options(
  cfg/camera.cfg
  #...
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES camera
#  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(SRC
"src/camera_node.cpp"
"src/main.cpp")

add_executable(camera_node ${SRC})
add_dependencies(camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(camera_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(camera_node
  ${catkin_LIBRARIES}
  libgxiapi.so
)

