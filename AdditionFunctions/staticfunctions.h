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
        static cv::Mat GenerateRotationMatrix(float, int);
        static cv::Mat getEulerAngles(const cv::Mat);
        static cv::Mat getRotationMatrix(const cv::Mat euler_angles);
        static cv::Point3d transformPoint3d(cv::Point3d point, cv::Mat R, cv::Point3d T);
    };
}
#endif // STATICFUNCTIONS_H
