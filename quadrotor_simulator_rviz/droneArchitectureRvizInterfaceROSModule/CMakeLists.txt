cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME droneArchitectureRvizROSModule)
project(${PROJECT_NAME})

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
##add_definitions(-std=c++11)
#add_definitions(-std=c++0x)
#add_definitions(-std=c++03)
ADD_DEFINITIONS(
    -std=c++11 # Or -std=c++0x
    #-std=c++03
    # Other flags
)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries



set(DRONEARCHITECTURERVIZINTERFACE_ROSMODULE_SOURCE_DIR
        src/source)

set(DRONEARCHITECTURERVIZINTERFACE_ROSMODULE_INCLUDE_DIR
        src/include)

set(DRONEARCHITECTURERVIZINTERFACE_ROSMODULE_HEADER_FILES
    ${DRONEARCHITECTURERVIZINTERFACE_ROSMODULE_INCLUDE_DIR}/swarmAgentInterface.h
    ${DRONEARCHITECTURERVIZINTERFACE_ROSMODULE_INCLUDE_DIR}/RvizInterfaceROSModule.h
    ${DRONEARCHITECTURERVIZINTERFACE_ROSMODULE_INCLUDE_DIR}/RvizInteractiveMarkerDisplay.h
)

set(DRONEARCHITECTURERVIZINTERFACE_ROSMODULE_SOURCE_FILES
    ${DRONEARCHITECTURERVIZINTERFACE_ROSMODULE_SOURCE_DIR}/swarmAgentInterface.cpp
    ${DRONEARCHITECTURERVIZINTERFACE_ROSMODULE_SOURCE_DIR}/RvizInterfaceROSModule.cpp
    ${DRONEARCHITECTURERVIZINTERFACE_ROSMODULE_SOURCE_DIR}/RvizInteractiveMarkerDisplay.cpp
)



find_package(catkin REQUIRED
                COMPONENTS roscpp std_msgs sensor_msgs visualization_msgs interactive_markers lib_cvgutils droneMsgsROS droneModuleROS)



catkin_package(
	CATKIN_DEPENDS roscpp std_msgs sensor_msgs visualization_msgs interactive_markers lib_cvgutils droneMsgsROS droneModuleROS
  )


include_directories(${DRONEARCHITECTURERVIZINTERFACE_ROSMODULE_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})



add_library(droneArchitectureRvizROSModule ${DRONEARCHITECTURERVIZINTERFACE_ROSMODULE_SOURCE_FILES} ${DRONEARCHITECTURERVIZINTERFACE_ROSMODULE_HEADER_FILES})
add_dependencies(droneArchitectureRvizROSModule ${catkin_EXPORTED_TARGETS})
target_link_libraries(droneArchitectureRvizROSModule ${catkin_LIBRARIES})


add_executable(droneArchitectureRvizInterfaceROSModuleNode ${DRONEARCHITECTURERVIZINTERFACE_ROSMODULE_SOURCE_DIR}/RvizInterfaceROSModuleNode.cpp)
add_dependencies(droneArchitectureRvizInterfaceROSModuleNode ${catkin_EXPORTED_TARGETS})
target_link_libraries(droneArchitectureRvizInterfaceROSModuleNode droneArchitectureRvizROSModule)
target_link_libraries(droneArchitectureRvizInterfaceROSModuleNode ${catkin_LIBRARIES})
