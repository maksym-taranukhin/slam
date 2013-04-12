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
      /// find the correspondences between two feature-points sets
      /**
          \param fp1 - first feature-points set
          \param fp2 - second feature-points set
          \return vector of correspondences (index in fp1 - index in fp2)
      */

      virtual std::vector< cv::DMatch > getMatches(FPoints* fp1, FPoints* fp2) = 0;// returns value, couse its small object
    };
}
#endif // IFEATUREMATCHING_H
