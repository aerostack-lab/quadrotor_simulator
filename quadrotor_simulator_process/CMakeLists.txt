cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME quadrotor_simulator_process)
project(${PROJECT_NAME})

### Use version C++03 of the C++ standard
add_definitions(-std=c++11)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries




set(QUADROTOR_SIMULATOR_SOURCE_DIR
        src/source)
	
set(QUADROTOR_SIMULATOR_INCLUDE_DIR
	src/include
	)

set(QUADROTOR_SIMULATOR_SOURCE_FILES
        ${QUADROTOR_SIMULATOR_SOURCE_DIR}/quadrotor_simulator_process.cpp
	)
	
set(QUADROTOR_SIMULATOR_HEADER_FILES
        ${QUADROTOR_SIMULATOR_INCLUDE_DIR}/quadrotor_simulator_process.h
	)


find_package(catkin REQUIRED
                COMPONENTS roscpp std_msgs geometry_msgs sensor_msgs aerostack_msgs tf_conversions robot_process quadrotor_simulator lib_cvgutils)


catkin_package(
        CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs aerostack_msgs tf_conversions robot_process quadrotor_simulator lib_cvgutils
  )


include_directories(${QUADROTOR_SIMULATOR_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})


add_library(quadrotor_simulator_process ${QUADROTOR_SIMULATOR_SOURCE_FILES} ${QUADROTOR_SIMULATOR_HEADER_FILES})
add_dependencies(quadrotor_simulator_process ${catkin_EXPORTED_TARGETS})
target_link_libraries(quadrotor_simulator_process ${catkin_LIBRARIES})


add_executable(quadrotor_simulator_process_main ${QUADROTOR_SIMULATOR_SOURCE_DIR}/quadrotor_simulator_process_main.cpp)
add_dependencies(quadrotor_simulator_process_main ${catkin_EXPORTED_TARGETS})
target_link_libraries(quadrotor_simulator_process_main quadrotor_simulator_process)
target_link_libraries(quadrotor_simulator_process_main ${catkin_LIBRARIES})


