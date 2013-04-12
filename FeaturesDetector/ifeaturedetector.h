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
        /// find the feature points for kinect frame
        /**
            \param frame - frame from kinect
            \return detected feature-points
        */

        virtual FPoints * getFeatures (const LuxFrame * frame) = 0;// FPoints * must be allocated
    };
}
#endif // IFEATUREDETECTOR_H
