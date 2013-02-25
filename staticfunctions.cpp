#include "staticfunctions.h"
namespace LuxSlam
{
    StaticFunctions::StaticFunctions()
    {
    }
    double StaticFunctions::distance2d(const cv::Point2d &p1, const cv::Point2d &p2)
    {
        double res = (p1.x - p2.x)*(p1.x - p2.x)+(p1.y - p2.y)*(p1.y - p2.y);
        return std::sqrt(res);
    }
    double StaticFunctions::distance3d(const cv::Point3d &p1, const cv::Point3d &p2)
    {
        double res = (p1.x - p2.x)*(p1.x - p2.x)+(p1.y - p2.y)*(p1.y - p2.y)+(p1.z - p2.z)*(p1.z - p2.z);
        return std::sqrt(res);

    }
}
