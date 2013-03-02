#include "featuredetectorsurf.h"

namespace LuxSlam
{
    FeatureDetectorSurf::FeatureDetectorSurf()
    {
    }
    FPoints * FeatureDetectorSurf::getFeatures (const LuxFrame * frame)
    {
        FPoints * fp = new FPoints();

        cv::Mat gray;

        cv::cvtColor(frame->image,gray,CV_BGR2GRAY);

        int minHessian = 350;

        cv::SurfFeatureDetector detector( minHessian );

        detector.detect( gray, fp->points);
        //-- Step 2: Calculate descriptors (feature vectors)
        cv::SurfDescriptorExtractor extractor;

        extractor.compute( gray, fp->points, fp->deskriptors );

        return fp;
    }
}
