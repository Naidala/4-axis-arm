cmake_minimum_required(VERSION 2.8.3)
project(joint_pkg)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  gcode_pkg
  sensor_msgs
  message_generation
)

#foreach(dir config launch meshes urdf)
#	install(DIRECTORY ${dir}/
#		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
#endforeach(dir)

add_message_files(FILES Data_Joints_EE.msg)
generate_messages(DEPENDENCIES sensor_msgs geometry_msgs)

catkin_package(
   CATKIN_DEPENDS message_runtime sensor_msgs geometry_msgs
)


include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/joint_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
