cmake_minimum_required(VERSION 2.8.3)

project(bimos)

# Finding headers and launch files for QtCreator
file(GLOB_RECURSE HDRS ${CMAKE_CURRENT_SOURCE_DIR} *.h *.hpp)
file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR} *.cxx *.cpp *.c *.py)
file(GLOB_RECURSE LNCH ${CMAKE_CURRENT_SOURCE_DIR} *.launch)

# Catkin dependencies
find_package(catkin REQUIRED COMPONENTS roscpp)

# System dependencies
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
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
    CATKIN_DEPENDS roscpp
    DEPENDS OpenCV Boost
)

###########
## Build ##
###########

# Including directories.
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Custom target to include headers and launch files in QtCreator
add_custom_target(dtarget_bimos SOURCES ${HDRS} ${SRCS} ${LNCH})

# MosaicBuilder Library
set(BIMOS_SRCS
    src/MosaicBuilder.cpp
    src/util/Params.cpp
    src/util/util.cpp
)
add_library(bimos ${BIMOS_SRCS})
target_link_libraries(bimos ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

# MosaicBuilder Node
add_executable(bimos_node src/node/bimos_node.cpp)
target_link_libraries(bimos_node bimos)
