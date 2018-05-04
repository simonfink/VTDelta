cmake_minimum_required(VERSION 2.6)

project(parallel-scara)

find_package(EEROS REQUIRED)
include_directories(${EEROS_INCLUDE_DIR};${EEROS_LIBUCL_INCLUDE_DIR})
link_directories(${EEROS_LIB_DIR};${EEROS_LIBUCL_LINK_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(parallel-scara 
	Main.cpp 
	control/ParallelScaraControlSystem.cpp
	safety/ParallelScaraSafetyProperties.cpp
	control/robotcontroller/RobotController.cpp
	control/pendulumcontrol/PendulumController.cpp
)
target_link_libraries(parallel-scara eeros ucl ${CMAKE_DL_LIBS})


# Test Programs

add_executable(test-hardware 
	test/hardware/TestHardware.cpp
)
target_link_libraries(test-hardware eeros ucl ${CMAKE_DL_LIBS})

add_executable(test-kinematic
	test/kinematic/testKin.cpp 
)
target_link_libraries(test-kinematic eeros ucl ${CMAKE_DL_LIBS})

add_executable(test-path
	test/pathplanner/testPath.cpp 
)
target_link_libraries(test-path eeros ucl ${CMAKE_DL_LIBS})
