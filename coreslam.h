#ifndef CORESLAM_H
#define CORESLAM_H
#include "opencv/highgui.h"
#include <iostream>

#include "FeaturesDetector/ifeaturedetector.h"
#include "FeaturesMatcher/ifeaturematching.h"
#include "TransformationCalculator/irtfinder.h"
#include "MatchesFilter/imatchesfilter.h"

#include "FeaturesDetector/featuredetectorsurf.h"
#include "FeaturesMatcher/featurematcherflann.h"
#include "TransformationCalculator/rtfindertrilateration.h"
#include "MatchesFilter/matchesfiltercvteam.h"
#include "luxframe.hpp"

#include "matchpoints.h"
#include "BundleAdjustment/opencvboundleadjustment.h"

namespace LuxSlam
{
    /** Sequence diagram for main function run()
        \msc
            hscale="1.5";
            CoreSlam, IFeatureDetector, IFeatureMatching, IMatchesFilter, IRTFinder, Logger;

            |||;

            CoreSlam->IFeatureDetector [label="getFeatures", URL="\ref IFeatureDetector::getFeatures()", ID="1"];
            CoreSlam<-IFeatureDetector [label="return features"];

            CoreSlam->IFeatureMatching [label="getMatches", URL="\ref IFeatureMatching::getMatches()", ID="2"];
            CoreSlam<-IFeatureMatching [label="return matches"];

            CoreSlam->IMatchesFilter [label="filterMatches", URL="\ref IMatchesFilter::filterMatches()", ID="3"];
            CoreSlam<-IMatchesFilter [label="return filtered matches"];

            CoreSlam->IRTFinder [label="calculate transformation", URL="\ref IRTFinder::getRTVector()", ID="4"];
            CoreSlam<-IRTFinder [label="return transformation matrix"];

            CoreSlam->Logger [label="get logger reference", URL="\ref Logget::getInstance()", ID="5"];
            CoreSlam<-Logger [label="return logger reference"];

            CoreSlam->Logger [label="logging some calculated data", URL="\ref Logget::logCamera()", ID="6"];
            CoreSlam<-Logger [label=""];



        \endmsc
     */
    class CoreSlam
    {
    public:
        /// default constructor
        CoreSlam();

        /// run slam algorithm
        /**
            \param frame - image from kinect
        */
        void run(LuxFrame* frame);
    private:
        IFeatureDetector *fdetector;
        IFeatureMatching *fmatcher;
        IRTFinder *rtfinder;
        IMatchesFilter *ffilter;
        LuxFrame *prev_frame;
        LuxFrame *curr_frame;
        FPoints *prev_features;
        FPoints *curr_features;
        OpenCVBoundleAdjustemnt* bundle_adjusment;
        Triple global_transformation_vector;

        /// get 3d coordinates of matches points
        /**
            \param matches - matche points
            \param points - output vector fill 2d and 3d coordinates for each match point
        */
        void Get3dPointsOfMatches(const std::vector<cv::DMatch>& matches,  std::vector<MatchPoints>& points);
    };
}
#endif // CORESLAM_H
