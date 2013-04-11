#ifndef MATCHPOINTS_H
#define MATCHPOINTS_H

#include "opencv/highgui.h"
namespace LuxSlam
{
    struct MatchPoints
	{
	    cv::Point3d first3d;
	    cv::Point3d second3d;
	    cv::Point2f first2d;
	    cv::Point2f second2d;

	};
}
#endif // MATCHPOINTS_H
