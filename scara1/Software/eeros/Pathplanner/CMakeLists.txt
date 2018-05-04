project(ScaraPathplanner)
cmake_minimum_required(VERSION 2.6)

## Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
endif()

SET(EEROS_BASEDIR "~/eeros-git")
SET(EEROS_INCLUDES "${EEROS_BASEDIR}/includes" "${EEROS_BASEDIR}/build-scara")
SET(EEROS_LIBRARY "${EEROS_BASEDIR}/build-scara/src/libeeros.so")

include_directories(${EEROS_INCLUDES} ${CMAKE_CURRENT_BINARY_DIR})

add_executable(pathplannerDemo pathplannerDemo.cpp 
							   PathPlanner.cpp)

target_link_libraries(pathplannerDemo ${EEROS_LIBRARY})