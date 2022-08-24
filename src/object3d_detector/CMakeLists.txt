cmake_minimum_required(VERSION 2.8.3)
project(object3d_detector)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  bayes_tracking
  bayes_people_tracker
  hololens_depth_data_receiver_msgs
  message_generation
)

find_package(PCL REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

add_message_files(
  FILES
  ClassificationResult.msg
  DetectionResult.msg
  DetectionResults.msg
)

add_service_files(
  FILES
  ClassifyClusters.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS bayes_tracking)

### Offline version
add_executable(object3d_detector src/object3d_detector.cpp src/svm.cpp)
target_link_libraries(object3d_detector ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(object3d_detector ${catkin_EXPORTED_TARGETS})
endif()

### Online version
add_executable(object3d_detector_ol src/object3d_detector_ol.cpp src/svm.cpp)
target_link_libraries(object3d_detector_ol ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(object3d_detector_ol ${catkin_EXPORTED_TARGETS})
endif()

### Service version
add_executable(object3d_detector_srv src/object3d_detector_srv.cpp src/svm.cpp)
target_link_libraries(object3d_detector_srv ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(object3d_detector_srv ${catkin_EXPORTED_TARGETS})
endif()
