cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(NewOfNewSlam CXX C)

set(sources coreslam.cpp featuredetectorsurf.cpp featurematcherflann.cpp ifeaturedetector.cpp ifeaturematching.cpp imatchesfilter.cpp irtfinder.cpp luxslam.cpp matchesfiltercvteam.cpp rtfindertrilateration.cpp staticfunctions.cpp triples.cpp debugger_interface.cpp)

find_package(OpenCV REQUIRED)
find_package(Qt4 REQUIRED QtCore QtGui QtOpenGL QtXml)
include(${QT_USE_FILE})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../KinectRecorder/src/)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(NewOfNewSlam SHARED ${sources} ${QT_LIBRARIES})
set(GLOBAL_LIBS ${GLOBAL_LIBS} NewOfNewSlam PARENT_SCOPE)


