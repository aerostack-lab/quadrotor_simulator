cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME droneSimulatorRvizROSModule)
project(${PROJECT_NAME})

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
#add_definitions(-std=c++11)
#add_definitions(-std=c++0x)
#ADD_DEFINITIONS(-std=c++11 # Or -std=c++0x
    # Other flags

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries



set(DRONERVIZINTERFACE_ROSMODULE_SOURCE_DIR
        src/source)

set(DRONERVIZINTERFACE_ROSMODULE_INCLUDE_DIR
        src/include)

set(DRONERVIZINTERFACE_ROSMODULE_HEADER_FILES
    ${DRONERVIZINTERFACE_ROSMODULE_INCLUDE_DIR}/SimulatorRvizROSModule.h
       
)

set(DRONERVIZINTERFACE_ROSMODULE_SOURCE_FILES
    ${DRONERVIZINTERFACE_ROSMODULE_SOURCE_DIR}/SimulatorRvizROSModule.cpp
        
)


find_package(catkin REQUIRED
        COMPONENTS roscpp std_msgs geometry_msgs visualization_msgs interactive_markers droneMsgsROS)



catkin_package(
        CATKIN_DEPENDS roscpp std_msgs geometry_msgs visualization_msgs interactive_markers droneMsgsROS
  )


include_directories(${DRONERVIZINTERFACE_ROSMODULE_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})



add_library(SimulatorRvizROSModule ${DRONERVIZINTERFACE_ROSMODULE_SOURCE_FILES} ${DRONERVIZINTERFACE_ROSMODULE_HEADER_FILES})
add_dependencies(SimulatorRvizROSModule ${catkin_EXPORTED_TARGETS})
target_link_libraries(SimulatorRvizROSModule ${catkin_LIBRARIES})


add_executable(SimulatorRvizROSModuleNode src/source/SimulatorRvizROSModuleNode.cpp)
add_dependencies(SimulatorRvizROSModuleNode ${catkin_EXPORTED_TARGETS})
target_link_libraries(SimulatorRvizROSModuleNode SimulatorRvizROSModule)
target_link_libraries(SimulatorRvizROSModuleNode ${catkin_LIBRARIES})

