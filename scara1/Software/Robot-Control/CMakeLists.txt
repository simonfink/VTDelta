cmake_minimum_required(VERSION 2.6)

project(NTB-SCARA)

message("CMake RobotControl")
## Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
endif()

add_subdirectory(test)

#SET(EEROS_INCLUDE_DIR "${EEROS_BASEDIR}/includes" "${EEROS_BASEDIR}/build-scara")
#SET(EEROS_LINK_DIR "${EEROS_BASEDIR}/build-scara/src/libeeros.so")


find_package(EEROS REQUIRED)
include_directories(${EEROS_INCLUDE_DIR};${EEROS_LIBUCL_INCLUDE_DIR})
link_directories(${EEROS_LIB_DIR};${EEROS_LIBUCL_LINK_DIR})

include_directories(${ADDITIONAL_INCLUDE_DIRS})
link_directories(${ADDITIONAL_LINK_DIRS})

#include_directories(${EEROS_INCLUDE_DIR} "${EEROS_LINK_DIR}/.." "${CMAKE_SOURCE_DIR}/includes")
#SET(EEROS_LIBRARY "${EEROS_LINK_DIR}/libeeros.so")
set(EEROS_LIBRARIES eeros ${CMAKE_DL_LIBS} ucl pthread)
#link_directories(${EEROS_LINK_DIR})

add_executable(robotControl 
							Main.cpp 
							ScaraSafetyProperties.cpp 
							ScaraControlSystem.cpp 
							control/kinematicDynamic/ScaraInverseKinematic.cpp 
							control/kinematicDynamic/ScaraDirectKinematic.cpp 
							control/kinematicDynamic/ScaraTransposeJacobi.cpp 
							control/Filter.cpp
							control/workspaceLimitation/CartesianWorkspaceLimitation.cpp
							joystick/XBoxInput.cpp 
							joystick/XBoxButtons.cpp 
							joystick/Joystick.cpp
							
							ScaraSequenceMain.cpp
							sequences/goingToReady/HomeToReady.cpp
							sequences/goingToReady/GoToReady.cpp
							sequences/moving/Move.cpp
							sequences/moving/Move_Joystick.cpp
)

target_link_libraries(robotControl ${EEROS_LIBRARIES})

add_executable(robotControl_force 
							Main_force.cpp 
							force_control/ScaraSafetyProperties_force.cpp 
							force_control/ScaraControlSystem_force.cpp 
							control/kinematicDynamic/ScaraInverseKinematic.cpp 
							control/kinematicDynamic/ScaraDirectKinematic.cpp
							control/kinematicDynamic/ScaraTransposeJacobi.cpp
							control/kinematicDynamic/DynModell.cpp
							control/Filter.cpp
							joystick/XBoxInput.cpp 
							joystick/XBoxButtons.cpp 
							joystick/Joystick.cpp
							
							force_control/ScaraSequenceMain_force.cpp
							force_control/sequences/goingToReady/HomeToReady.cpp
							force_control/sequences/goingToReady/GoToReady.cpp
							force_control/sequences/moving/Move.cpp
							force_control/sequences/moving/Move_Joystick.cpp
							force_control/sequences/forceControl/ForceControl.cpp
)

target_link_libraries(robotControl_force ${EEROS_LIBRARIES})

# add_executable(testPath 
# 							test/newPathPlanner/TestPP.cpp 
# 							test/newPathPlanner/PPControlSystem.cpp 
# 							test/newPathPlanner/PPSafetyProperties.cpp 
# )
# 
# target_link_libraries(testPath ${EEROS_LIBRARIES})
# 
# add_executable(testEndSwitches 
# 							test/endSwitches/TestEndSwitches.cpp 
# 							test/endSwitches/ESControlSystem.cpp 
# 							test/endSwitches/ESSafetyProperties.cpp 
# )
# 
# target_link_libraries(testEndSwitches ${EEROS_LIBRARIES})
# 
# add_executable(testEncoder 
# 							test/encoder/TestEncoder.cpp 
# 							test/encoder/EncControlSystem.cpp 
# 							test/encoder/EncSafetyProperties.cpp 
# )
# 
# target_link_libraries(testEncoder ${EEROS_LIBRARIES})
# 
# add_executable(testForceSensor 
# 							test/forceSensor/TestForceSensor.cpp 
# 							test/forceSensor/FSControlSystem.cpp 
# 							test/forceSensor/FSSafetyProperties.cpp 
# )
# 
# target_link_libraries(testForceSensor ${EEROS_LIBRARIES})