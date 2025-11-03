cmake_minimum_required(VERSION 3.0.2)
project(perception)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(camera_data src/camera_data.cpp)
target_link_libraries(camera_data
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(pcl src/pcl_show.cpp)
target_link_libraries(pcl
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)




add_executable(cuboid src/voxelise.cpp)
target_link_libraries(cuboid
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)



