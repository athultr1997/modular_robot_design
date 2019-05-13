cmake_minimum_required(VERSION 2.8.3)
project(surf_rec)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  interactive_markers
  visualization_msgs
  tf
)

catkin_package(
 CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs interactive_markers visualization_msgs tf
 INCLUDE_DIRS
 include
)

include_directories(include ${catkin_INCLUDE_DIRS})

# install(DIRECTORY include/${PROJECT_NAME}/
# 	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

add_executable(point_cloud_generator src/point_cloud_generator.cpp)
target_link_libraries(point_cloud_generator ${catkin_LIBRARIES})
# add_dependencies(point_cloud_generator surf_rec_generate_messages_cpp)

add_executable(point_cloud_visualizer src/point_cloud_visualizer.cpp)
target_link_libraries(point_cloud_visualizer ${catkin_LIBRARIES})
# add_dependencies(point_cloud_visualizer surf_rec_generate_messages_cpp)

add_executable(triangulation src/triangulation.cpp)
target_link_libraries(triangulation ${catkin_LIBRARIES})
# add_dependencies(triangulation surf_rec_generate_messages_cpp)

add_executable(save_pclmsg_pcdfile src/save_pclmsg_pcdfile.cpp)
target_link_libraries(save_pclmsg_pcdfile ${catkin_LIBRARIES})
# add_dependencies(save_pclmsg_pcdfile surf_rec_generate_messages_cpp)

add_executable(revolution_command src/revolution_command.cpp)
target_link_libraries(revolution_command ${catkin_LIBRARIES})
# add_dependencies(revolution_command surf_rec_generate_messages_cpp)

add_executable(point_cloud_assembler src/point_cloud_assembler.cpp)
target_link_libraries(point_cloud_assembler ${catkin_LIBRARIES})
# add_dependencies(point_cloud_assembler surf_rec_generate_messages_cpp)

add_executable(point_cloud_frame_change src/point_cloud_frame_change.cpp)
target_link_libraries(point_cloud_frame_change ${catkin_LIBRARIES})
# add_dependencies(point_cloud_frame_change surf_rec_generate_messages_cpp)

add_executable(rviz_user_input src/rviz_user_input.cpp)
target_link_libraries(rviz_user_input ${catkin_LIBRARIES})
# add_dependencies(rviz_user_input surf_rec_generate_messages_cpp)

add_executable(rec_surf_generator src/rec_surf_generator.cpp)
target_link_libraries(rec_surf_generator ${catkin_LIBRARIES})

add_executable(urdf_generator src/urdf_generator.cpp)
target_link_libraries(urdf_generator ${catkin_LIBRARIES})
