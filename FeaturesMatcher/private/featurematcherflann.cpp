#include "../featurematcherflann.h"
namespace LuxSlam
{
    FeatureMatcherFlann::FeatureMatcherFlann()
    {
    }
    std::vector< cv::DMatch > FeatureMatcherFlann::getMatches(FPoints* pts1, FPoints* pts2)
    {

        std::vector< cv::DMatch > matches;
        cv::FlannBasedMatcher matcher;

        matcher.match( pts1->deskriptors, pts2->deskriptors, matches );

        return matches;
    }
}
