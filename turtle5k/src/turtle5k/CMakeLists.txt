cmake_minimum_required(VERSION 2.8.3)
project(turtle5k)

find_package(
	catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
)

catkin_package(

)

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(strategy src/Strategy.cpp)
target_link_libraries(strategy ${catkin_LIBRARIES})
add_dependencies(strategy turtle5k_generate_messages_cpp)

add_executable(shooting src/Shooting.cpp)
target_link_libraries(shooting ${catkin_LIBRARIES})
add_dependencies(shooting turtle5k_generate_messages_cpp)

add_executable(wheelcontrol src/WheelControl.cpp)
target_link_libraries(wheelcontrol ${catkin_LIBRARIES})
add_dependencies(wheelcontrol turtle5k_generate_messages_cpp)

add_executable(pathfinding src/Pathfinding.cpp)
target_link_libraries(pathfinding ${catkin_LIBRARIES})
add_dependencies(pathfinding turtle5k_generate_messages_cpp)

add_executable(ballhandling src/BallHandling.cpp)
target_link_libraries(ballhandling ${catkin_LIBRARIES})
add_dependencies(ballhandling turtle5k_generate_messages_cpp)

add_executable(world src/World.cpp)
target_link_libraries(world ${catkin_LIBRARIES})
add_dependencies(world turtle5k_generate_messages_cpp)

add_executable(localisation src/Localisation.cpp)
target_link_libraries(localisation ${catkin_LIBRARIES})
add_dependencies(localisation turtle5k_generate_messages_cpp)

add_executable(ball_detection src/Ball_Detection.cpp)
target_link_libraries(ball_detection ${catkin_LIBRARIES})
add_dependencies(ball_detection turtle5k_generate_messages_cpp)
