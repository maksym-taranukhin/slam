#ifndef IFEATUREMATCHING_H
#define IFEATUREMATCHING_H
#include "../FeaturesDetector/ifeaturedetector.h"
#include "opencv2/features2d/features2d.hpp"
namespace LuxSlam
{
    class IFeatureMatching
    {
    public:
      IFeatureMatching();
      virtual std::vector< cv::DMatch > getMatches(FPoints*, FPoints*) = 0;// returns value, couse its small object
    };
}
#endif // IFEATUREMATCHING_H
