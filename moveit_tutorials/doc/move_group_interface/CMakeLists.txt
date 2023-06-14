add_executable(move_group_interface_tutorial src/move_group_interface_tutorial.cpp)
add_executable(move_group_cpp_trajectory_generation src/move_group_cpp_trajectory_generation.cpp)

add_executable(move_group_data_collection src/move_group_data_collection.cpp)

target_link_libraries(move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

target_link_libraries(move_group_cpp_trajectory_generation ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_cpp_trajectory_generation DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

target_link_libraries(move_group_data_collection ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_data_collection DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
