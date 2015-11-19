cmake_minimum_required(VERSION 2.8.3)
project(turtle5k)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

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

add_executable(strategy src/strategy.cpp)
target_link_libraries(strategy ${catkin_LIBRARIES})
add_dependencies(strategy turtle5k_generate_messages_cpp)

add_executable(tactics src/tactics.cpp)
target_link_libraries(tactics ${catkin_LIBRARIES})
add_dependencies(tactics turtle5k_generate_messages_cpp)

add_executable(pathfinding src/pathfinding.cpp)
target_link_libraries(pathfinding ${catkin_LIBRARIES})
add_dependencies(pathfinding turtle5k_generate_messages_cpp)

add_executable(commands src/commands.cpp)
target_link_libraries(commands ${catkin_LIBRARIES})
add_dependencies(commands turtle5k_generate_messages_cpp)

add_executable(world src/world.cpp)
target_link_libraries(world ${catkin_LIBRARIES})
add_dependencies(world turtle5k_generate_messages_cpp)

add_executable(ballcontrol src/ballcontrol.cpp)
target_link_libraries(ballcontrol ${catkin_LIBRARIES})
add_dependencies(ballcontrol turtle5k_generate_messages_cpp)

add_executable(wheelcontrol src/wheelcontrol.cpp)
target_link_libraries(wheelcontrol ${catkin_LIBRARIES})
add_dependencies(wheelcontrol turtle5k_generate_messages_cpp)

add_executable(positioning src/positioning.cpp)
target_link_libraries(positioning ${catkin_LIBRARIES})
add_dependencies(positioning turtle5k_generate_messages_cpp)

add_executable(balldetector src/balldetector.cpp)
target_link_libraries(balldetector ${catkin_LIBRARIES})
add_dependencies(balldetector turtle5k_generate_messages_cpp)

add_executable(goaldetector src/goaldetector.cpp)
target_link_libraries(goaldetector ${catkin_LIBRARIES})
add_dependencies(goaldetector turtle5k_generate_messages_cpp)

add_executable(ballhandling src/positioning.cpp)
target_link_libraries(ballhandling ${catkin_LIBRARIES})
add_dependencies(ballhandling turtle5k_generate_messages_cpp)

add_executable(shooting src/shooting.cpp)
target_link_libraries(shooting ${catkin_LIBRARIES})
add_dependencies(shooting turtle5k_generate_messages_cpp)

add_executable(compass src/compass.cpp)
target_link_libraries(compass ${catkin_LIBRARIES})
add_dependencies(compass turtle5k_generate_messages_cpp)

add_executable(camera src/camera.cpp)
target_link_libraries(camera ${catkin_LIBRARIES})
add_dependencies(camera turtle5k_generate_messages_cpp)

# Hardware source
file(GLOB Library_SOURCES src/hardware/*.cpp)
file(GLOB Library_HEADER src/hardware/*.h)

# Ball handling
add_executable(ballhandlingStub src/stubs/ballhandlingStub.cpp ${Library_SOURCES} ${Library_HEADER})
target_link_libraries(ballhandlingStub ${catkin_LIBRARIES})
add_dependencies(ballhandlingStub turtle5k_generate_messages_cpp)

# Compass
add_executable(compassStub src/stubs/compassStub.cpp ${Library_SOURCES} ${Library_HEADER})
target_link_libraries(compassStub ${catkin_LIBRARIES})
add_dependencies(compassStub turtle5k_generate_messages_cpp)
