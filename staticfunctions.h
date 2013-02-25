#ifndef STATICFUNCTIONS_H
#define STATICFUNCTIONS_H
#include "opencv/highgui.h"
namespace LuxSlam
{
    class StaticFunctions
    {
    public:
        StaticFunctions();
        static double distance2d(const cv::Point2d &p1, const cv::Point2d &p2);
        static double distance3d(const cv::Point3d &p1, const cv::Point3d &p2);

    };
}
#endif // STATICFUNCTIONS_H
