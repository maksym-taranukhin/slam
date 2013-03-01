cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(NewOfNewSlam CXX C)

set(sources coreslam.cpp featuredetectorsurf.cpp featurematcherflann.cpp ifeaturedetector.cpp ifeaturematching.cpp imatchesfilter.cpp irtfinder.cpp luxslam.cpp matchesfiltercvteam.cpp rtfindertrilateration.cpp staticfunctions.cpp triples.cpp debugger_interface.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(NewOfNewSlam SHARED ${sources} ${QT_LIBRARIES})

if (DEFINED GLOBAL_LIBS)
	set(GLOBAL_LIBS ${GLOBAL_LIBS} NewOfNewSlam; PARENT_SCOPE)
endif()

if (DEFINED ALGO_INCLUDES)
	set(ALGO_INCLUDES "${ALGO_INCLUDES}#include <debugger_interface.hpp>\n" PARENT_SCOPE)
endif ()

if (DEFINED ALGO_CODE)
	set(ALGO_CODE ${ALGO_CODE}"IAbstractAlgorithm * slam = new SlamInterface(config); create(slam);\n"
		PARENT_SCOPE)
endif()
