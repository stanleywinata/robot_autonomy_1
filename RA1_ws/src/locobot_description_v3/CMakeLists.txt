cmake_minimum_required(VERSION 2.8.3)

project(locobot_description_v3)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  urdf
)

catkin_package(
  # CATKIN_DEPENDS urdf
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(parser src/parser.cpp)
add_executable(forkin src/forward_kinematics.cpp) 
target_link_libraries(parser ${catkin_LIBRARIES})
set (CMAKE_XXX_STANDARD_11)
foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
