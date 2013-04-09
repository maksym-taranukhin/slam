#ifndef FEATUREDETECTORSURF_H
#define FEATUREDETECTORSURF_H
#include "ifeaturedetector.h"
namespace LuxSlam
{
    class FeatureDetectorSurf : public IFeatureDetector
    {
    public:
        FeatureDetectorSurf();
        FPoints * getFeatures (const LuxFrame *);
    };
}
#endif // FEATUREDETECTORSURF_H
