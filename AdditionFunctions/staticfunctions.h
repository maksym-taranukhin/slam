#ifndef STATICFUNCTIONS_H
#define STATICFUNCTIONS_H
#include "opencv/highgui.h"
namespace LuxSlam
{
    class StaticFunctions
    {
    public:
        StaticFunctions();
        /// compute the distatnce between 2D points
        /**
            \param p1 - first point
            \param p2 - second point
            \return distanse
        */
        static double distance2d(const cv::Point2d &p1, const cv::Point2d &p2);

        /// compute the distatnce between 3D points
        /**
            \param p1 - first point
            \param p2 - second point
            \return distanse
        */
        static double distance3d(const cv::Point3d &p1, const cv::Point3d &p2);

        /// generate the rotation matrix for axis and angle, defined in parameters
        /**
            \param angle    - angle in radians
            \param axis     - number of axis:
                1 - x-axis;
                2 - y-axis;
                3 - z-axis.
            \return [3X3] rotation matrix
        */
        static cv::Mat GenerateRotationMatrix(float angle, int axis);

        /// compute the euler angles for rotation matrix
        /**
            \param R - rotation matrix
            \return euler angles in [3X1] matrix
        */
        static cv::Mat getEulerAngles(const cv::Mat R);

        /// compute the rotation matrix for given euler angles
        /**
            \param euler_angles - euler_angles in [3X1] matrix
            \return [3X3] rotation matrix
        */
        static cv::Mat getRotationMatrix(const cv::Mat euler_angles);

        /// apply affine transformation for the point in 3D
        /**
            \param point - point in 3D
            \param R - [3X3] rotation matrix
            \param T - traslation vector in cv::Point3d
            \return point after transformation
        */

        static cv::Point3d transformPoint3d(cv::Point3d point, cv::Mat R, cv::Point3d T);
    };
}
#endif // STATICFUNCTIONS_H
