cmake_minimum_required(VERSION 2.6)

project(NTB-SCARA)

## Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
endif()

#SET(EEROS_INCLUDE_DIR "${EEROS_BASEDIR}/includes" "${EEROS_BASEDIR}/build-scara")
#SET(EEROS_LINK_DIR "${EEROS_BASEDIR}/build-scara/src/libeeros.so")

include_directories(${EEROS_INCLUDE_DIR} "${EEROS_LINK_DIR}/.." "${CMAKE_SOURCE_DIR}/includes")
SET(EEROS_LIBRARY "${EEROS_LINK_DIR}/libeeros.so")
#link_directories(${EEROS_LINK_DIR})

add_executable(robotControl 
							Main.cpp 
							ScaraSafetyProperties.cpp 
							ScaraControlSystem.cpp 
							control/kinematicDynamic/ScaraInverseKinematic.cpp 
							control/kinematicDynamic/ScaraDirectKinematic.cpp 
							control/pathPlanner/PathPlanner.cpp
							control/workspaceLimitation/CartesianWorkspaceLimitation.cpp
							control/SetPlasmaPen.cpp
							joystick/XBoxInput.cpp 
							joystick/XBoxButtons.cpp 
							joystick/Joystick.cpp
							ScaraSequenceMain.cpp 
							sequences/parking/ScaraSequenceManualPark.cpp
							sequences/homing/ScaraSequenceHoming.cpp
							sequences/goingToReady/ScaraSequenceHomeToReady.cpp 
							sequences/referenceSystems/ScaraSequenceLoadRefSystem.cpp
							sequences/workspace/ScaraSequenceLoadWorkspaceLimitation.cpp 
							sequences/tools/ScaraSequenceGoToToolChange.cpp
							sequences/goingToReady/ScaraSequenceGoToReady.cpp 
							sequences/referenceSystems/ScaraSequenceCalculateRefSystem.cpp
							sequences/referenceSystems/ScaraSequenceSaveRefSystems.cpp
							sequences/tools/ScaraSequenceCheckTool.cpp
							sequences/manual/ScaraSequenceManual.cpp
							sequences/calibration/ScaraSequenceSaveCalibratedMesh.cpp
							sequences/calibration/ScaraSequenceSaveCalibratedPartsStorage.cpp
							sequences/texass/ScaraSequenceTEXASS.cpp 
							sequences/texass/plasmapen/ScaraSequenceTEXASSplasmapen.cpp 
							sequences/texass/plasmapen/ScaraSequenceTEXASSsinglePlasmaLine.cpp 
							sequences/texass/plasmapen/ScaraSequenceTEXASSsinglePlasmaPoint.cpp 
							sequences/texass/plasmapen/ScaraSequenceTEXASSplasmaProcess.cpp
							sequences/texass/grabber/ScaraSequenceTEXASSgrabber.cpp 
							sequences/texass/grabber/ScaraSequenceTEXASSsingleGrab.cpp 
							sequences/texass/grabber/ScaraSequenceTEXASSgrabProcess.cpp 
							sequences/texass/dispencer/ScaraSequenceTEXASSdispencer.cpp 
							sequences/texass/dispencer/ScaraSequenceTEXASSsingleDispence.cpp 
							sequences/texass/dispencer/ScaraSequenceTEXASSdispenceProcess.cpp 
							
							sequences/ScaraSequenceMove.cpp
							sequences/parking/ScaraSequenceAutoPark.cpp 
# 							sequences/emergency/ScaraSequenceEmergencyReset.cpp 
)

target_link_libraries(robotControl ${EEROS_LIBRARY})

add_subdirectory(test)