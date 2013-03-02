#ifndef CORESLAM_H
#define CORESLAM_H
#include "opencv/highgui.h"
#include <iostream>

#include "ifeaturedetector.h"
#include "ifeaturematching.h"
#include "irtfinder.h"
#include "imatchesfilter.h"

#include "featuredetectorsurf.h"
#include "featurematcherflann.h"
#include "rtfindertrilateration.h"
#include "matchesfiltercvteam.h"
#include "luxframe.hpp"
#include "matchpoints.h"

namespace LuxSlam
{

    class CoreSlam
    {
    public:
        CoreSlam();
        void run(LuxFrame*);
    private:
        IFeatureDetector *fdetector;
        IFeatureMatching *fmatcher;
        IRTFinder *rtfinder;
        IMatchesFilter *ffilter;
        LuxFrame *prev_frame;
        LuxFrame *curr_frame;
        FPoints *prev_features;
        FPoints *curr_features;
        Triple global_rotation_translation_vector;
        void Get3dPointsOfMatches(const std::vector<cv::DMatch>& matches,  std::vector<MatchPoints>& points);
    };
}
#endif // CORESLAM_H
