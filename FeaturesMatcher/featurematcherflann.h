#ifndef FEATUREMATCHERFLANN_H
#define FEATUREMATCHERFLANN_H
#include "ifeaturematching.h"
#include "opencv2/nonfree/features2d.hpp"

namespace LuxSlam
{
    class FeatureMatcherFlann : public IFeatureMatching
    {
    public:
        FeatureMatcherFlann();
        std::vector< cv::DMatch > getMatches(FPoints*, FPoints*);
    };
}
#endif // FEATUREMATCHERFLANN_H
