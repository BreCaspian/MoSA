cmake_minimum_required(VERSION 3.8)
project(mosa)

set(CMAKE_BUILD_TYPE "Release")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-O3 -fexceptions)

message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processor number: ${N}")
  if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message("core for MP: 3")
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
    message("core for MP: ${PROC_NUM}")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(TBB REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(dynfilter
  src/dynfilter_with_odom.cpp
  src/DynObjFilter.cpp
  src/DynObjCluster.cpp
)
ament_target_dependencies(dynfilter
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
  pcl_conversions
  livox_interfaces
)
target_link_libraries(dynfilter ${PCL_LIBRARIES} ${OpenCV_LIBS} TBB::tbb)

add_executable(display_prediction src/display_prediction.cpp)
ament_target_dependencies(display_prediction
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
  pcl_conversions
  livox_interfaces
)
target_link_libraries(display_prediction ${PCL_LIBRARIES} ${OpenCV_LIBS} TBB::tbb)

add_executable(cal_recall src/cal_recall_multi.cpp)
ament_target_dependencies(cal_recall
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
  pcl_conversions
  livox_interfaces
)
target_link_libraries(cal_recall ${PCL_LIBRARIES} ${OpenCV_LIBS} TBB::tbb)

install(TARGETS
  dynfilter
  display_prediction
  cal_recall
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
  pcl_conversions
  livox_interfaces
)

ament_package()
