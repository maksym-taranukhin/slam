#ifndef IFEATUREDETECTOR_H
#define IFEATUREDETECTOR_H
#include "luxframe.hpp"
#include "opencv/highgui.h"
#include "opencv/cv.h"
#include "opencv2/nonfree/features2d.hpp"

namespace LuxSlam
{

    struct FPoints
    {
        std::vector<cv::KeyPoint> points;
        cv::Mat deskriptors;
    };

    class IFeatureDetector
    {
    public:
        IFeatureDetector();
        virtual FPoints * getFeatures (const LuxFrame *) = 0;// FPoints * must be allocated
    };
}
#endif // IFEATUREDETECTOR_H
