cmake_minimum_required(VERSION 2.8.3)

project(bimos)

# Finding headers and launch files for QtCreator
file(GLOB_RECURSE HDRS ${CMAKE_CURRENT_SOURCE_DIR} *.h *.hpp)
file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR} *.cxx *.cpp *.c *.py)
file(GLOB_RECURSE LNCH ${CMAKE_CURRENT_SOURCE_DIR} *.launch)

# Catkin dependencies
find_package(catkin REQUIRED COMPONENTS roscpp camera_calibration_parsers sensor_msgs image_transport cv_bridge obindex std_srvs dynamic_reconfigure)

# Dynamic reconfigure configuration
generate_dynamic_reconfigure_options(
    cfg/Bimos.cfg
)

# System dependencies
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Defining the package
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES bimos
    CATKIN_DEPENDS roscpp camera_calibration_parsers sensor_msgs image_transport cv_bridge obindex std_srvs dynamic_reconfigure
    DEPENDS OpenCV Boost Ceres OpenMP
)

###########
## Build ##
###########

# Including directories.
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

# Custom target to include headers and launch files in QtCreator
add_custom_target(dtarget_bimos SOURCES ${HDRS} ${SRCS} ${LNCH})

# BIMOS sources
set(BIMOS_SRCS
    include/bimos/util/ConcurrentQueue.hpp
    src/blend/Blender.cpp
    src/graph/Graph.cpp
    src/graph/MosaicGraph.cpp
    src/imgdesc/ImageDescriptor.cpp
    src/imgdesc/ldb.cpp
    src/imgdesc/ORBextractor.cc
    src/kfsel/KeyframeSelector.cpp
    src/loopcloser/LoopCloser.cpp
    src/motionest/AffineEstimator2D.cpp
    src/motionest/HomographyEstimator.cpp
    src/motionest/SimilarityEstimator2D.cpp
    src/optim/MosaicAdjuster.cpp
    src/optim/Optimizer.cpp
    src/util/MosaicPublisher.cpp
    src/util/Transform.cpp
    src/util/Params.cpp
    src/util/util.cpp
    src/node/bimos_node.cpp
)

# BIMOS Node
add_executable(bimos_node ${BIMOS_SRCS})
add_dependencies(bimos_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(bimos_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${CERES_LIBRARIES})

# BIMOS Rotational
add_executable(bimos_rotational
    src/util/Params.cpp
    src/util/util.cpp
    src/node/bimos_rotational.cpp)
add_dependencies(bimos_rotational ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(bimos_rotational ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

# BIMOS Conv Factors
add_executable(bimos_convfactors
    src/util/Params.cpp
    src/util/util.cpp
    src/node/bimos_convfactors.cpp)
add_dependencies(bimos_convfactors ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(bimos_convfactors ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

# Image Publisher Node
add_executable(image_publisher
    src/util/util.cpp
    src/node/image_publisher_node.cpp)
add_dependencies(image_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(image_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
