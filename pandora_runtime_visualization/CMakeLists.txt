cmake_minimum_required(VERSION 2.8.3)
project(pandora_runtime_visualization)

find_package(catkin REQUIRED COMPONENTS
      roscpp
      pandora_data_fusion_msgs
      sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
    #DEPENDS
    CATKIN_DEPENDS
      roscpp
      pandora_data_fusion_msgs
      sensor_msgs
    #INCLUDE_DIRS
    #LIBRARIES
)


add_executable(data_fusion_object_visualization src/data_fusion_object_visualization.cpp)
target_link_libraries(data_fusion_object_visualization ${catkin_LIBRARIES} )
add_dependencies(data_fusion_object_visualization ${catkin_EXPORTED_TARGETS})

add_executable(temprature_visualization src/temprature_visualization.cpp)
target_link_libraries(temprature_visualization ${catkin_LIBRARIES} )
