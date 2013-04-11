cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(NewOfNewSlam CXX C)


set(sources 			coreslam.cpp

                                FeaturesDetector/featuredetectorsurf.h
                                FeaturesDetector/ifeaturedetector.h
                                FeaturesDetector/private/featuredetectorsurf.cpp
                                FeaturesDetector/private/ifeaturedetector.cpp

                                FeaturesMatcher/featurematcherflann.h
                                FeaturesMatcher/ifeaturematching.h
                                FeaturesMatcher/private/featurematcherflann.cpp
                                FeaturesMatcher/private/ifeaturematching.cpp

                                MatchesFilter/imatchesfilter.h
                                MatchesFilter/matchesfiltercvteam.h
                                MatchesFilter/private/imatchesfilter.cpp
                                MatchesFilter/private/matchesfiltercvteam.cpp

                                TransformationCalculator/irtfinder.h
                                TransformationCalculator/rtfindertrilateration.h
                                TransformationCalculator/private/irtfinder.cpp
                                TransformationCalculator/private/rtfindertrilateration.cpp

                                BundleAdjustment/iboundleadjustment.h
                                BundleAdjustment/opencvboundleadjustment.h
                                BundleAdjustment/private/iboundleadjustment.cpp
                                BundleAdjustment/private/opencvboundleadjustment.cpp

                                AdditionFunctions/private/staticfunctions.cpp
                                AdditionFunctions/staticfunctions.h

				triples.h 
				
				matchpoints.h

				debugger_interface.cpp 

	)

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
