cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME quadrotor_simulator)
project(${PROJECT_NAME})

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
#add_definitions(-std=c++11)
#add_definitions(-std=c++0x)
add_definitions(-std=c++03)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries



set(DRONESIMULATOR_SOURCE_DIR
        src/source)

set(DRONESIMULATOR_INCLUDE_DIR
        src/include)

set(DRONESIMULATOR_HEADER_FILES
        # ARDroneType Simulator
        ${DRONESIMULATOR_INCLUDE_DIR}/ARDroneType/droneElements.h
        ${DRONESIMULATOR_INCLUDE_DIR}/ARDroneType/quadrotor_simulator.h
        ${DRONESIMULATOR_INCLUDE_DIR}/ARDroneType/quadrotor_model.h
        # AscTecPelicanType Simulator
        ${DRONESIMULATOR_INCLUDE_DIR}/AscTecPelicanType/pelican_like_model.h
        ${DRONESIMULATOR_INCLUDE_DIR}/AscTecPelicanType/pelican_like_simulator.h
        ${DRONESIMULATOR_INCLUDE_DIR}/AscTecPelicanType/pelican_like_elements.h
)

set(DRONESIMULATOR_SOURCE_FILES
        # ARDroneType Simulator
        ${DRONESIMULATOR_SOURCE_DIR}/ARDroneType/droneElements.cpp
        ${DRONESIMULATOR_SOURCE_DIR}/ARDroneType/quadrotor_simulator.cpp
        ${DRONESIMULATOR_SOURCE_DIR}/ARDroneType/quadrotor_model.cpp
        # AscTecPelicanType Simulator
        ${DRONESIMULATOR_SOURCE_DIR}/AscTecPelicanType/pelican_like_model.cpp
        ${DRONESIMULATOR_SOURCE_DIR}/AscTecPelicanType/pelican_like_simulator.cpp
        ${DRONESIMULATOR_SOURCE_DIR}/AscTecPelicanType/pelican_like_elements.cpp
)


find_package(catkin REQUIRED
                COMPONENTS lib_cvgutils lib_cvgekf)

find_package(OpenCV REQUIRED)


catkin_package(
        INCLUDE_DIRS ${DRONESIMULATOR_INCLUDE_DIR}
        LIBRARIES quadrotor_simulator
	DEPENDS OpenCV
        CATKIN_DEPENDS lib_cvgutils lib_cvgekf
  )


include_directories(${DRONESIMULATOR_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})



# quadrotor_simulator library compilation
add_library(quadrotor_simulator ${DRONESIMULATOR_SOURCE_FILES} ${DRONESIMULATOR_HEADER_FILES})
add_dependencies(quadrotor_simulator ${catkin_EXPORTED_TARGETS})
target_link_libraries(quadrotor_simulator ${OpenCV_LIBS})
target_link_libraries(quadrotor_simulator ${catkin_LIBRARIES})

