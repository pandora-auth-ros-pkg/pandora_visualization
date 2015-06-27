#~ find_package(roslib REQUIRED)
#~ find_package(rostest REQUIRED)
find_package(pandora_testing_tools REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${pandora_testing_tools_INCLUDE_DIRS}
  )

########################  Unit Tests  ###############################

########################  geotiff_creator  ####################################

catkin_add_gtest(geotiff_creator_test unit/geotiff_creator_test.cpp)
target_link_libraries(geotiff_creator_test
  ${catkin_LIBRARIES}
  ${pandora_testing_tools_LIBRARIES}
  geotiff_creator
  gtest_main
  )
