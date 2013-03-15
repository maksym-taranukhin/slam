#include "staticfunctions.h"
#include <iostream>
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

    cv::Mat StaticFunctions::getRotationMatrix(float angle, int axis)
    {
        cv::Mat mat(3,3,CV_32FC1);
        switch (axis) {
        case 1:
            mat.at<float>(0,0) = 1;
            mat.at<float>(1,0) = 0;
            mat.at<float>(2,0) = 0;
            mat.at<float>(0,1) = 0;
            mat.at<float>(1,1) = cos(angle);
            mat.at<float>(2,1) = sin(angle);
            mat.at<float>(0,2) = 0;
            mat.at<float>(1,2) = -sin(angle);
            mat.at<float>(2,2) = cos(angle);
            break;
        case 2:
            mat.at<float>(0,0) = cos(angle);
            mat.at<float>(1,0) = 0;
            mat.at<float>(2,0) = -sin(angle);
            mat.at<float>(0,1) = 0;
            mat.at<float>(1,1) = 1;
            mat.at<float>(2,1) = 0;
            mat.at<float>(0,2) = sin(angle);
            mat.at<float>(1,2) = 0;
            mat.at<float>(2,2) = cos(angle);
            break;
        default:
            mat.at<float>(0,0) = cos(angle);
            mat.at<float>(1,0) = sin(angle);
            mat.at<float>(2,0) = 0;
            mat.at<float>(0,1) = -sin(angle);
            mat.at<float>(1,1) = cos(angle);
            mat.at<float>(2,1) = 0;
            mat.at<float>(0,2) = 0;
            mat.at<float>(1,2) = 0;
            mat.at<float>(2,2) = 1;
            break;
        }

       // std::cerr << "getMatRot: " << mat << "\n";
         return mat;
    }

    cv::Mat StaticFunctions::getEulerAngles(const cv::Mat rotationMatrix)
    {
        cv::Mat eulerAngles(3,1, CV_32FC1);

        float r21 = rotationMatrix.at<float>(2,1);
        float r22 = rotationMatrix.at<float>(2,2);
        float r20 = rotationMatrix.at<float>(2,0);
        float r10 = rotationMatrix.at<float>(1,0);
        float r00 = rotationMatrix.at<float>(0,0);

        eulerAngles.at<float>(0,0) = std::atan2(r21, r22);
        eulerAngles.at<float>(1,0) = std::atan2(-r20, std::sqrt(r21*r21+r22*r22));
        eulerAngles.at<float>(2,0) = std::atan2(r10, r00);

        return eulerAngles;
    }

}
