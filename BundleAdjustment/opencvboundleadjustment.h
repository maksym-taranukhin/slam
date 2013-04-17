#ifndef OPENCV_BOUNDLE_ADJUSTMENT_H
#define OPENCV_BOUNDLE_ADJUSTMENT_H

#include "iboundleadjustment.h"
#include "queue"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "../AdditionFunctions/staticfunctions.h"

namespace LuxSlam
{
class OpenCVBoundleAdjustemnt : IBoundleAdjustment
{
public:
    OpenCVBoundleAdjustemnt(int numberOfCameras): numCamera(numberOfCameras)
    {
        //todo IN PARAMETRS constructor
        cameraRes = cv::Size(640, 480);
        cameraMat = (cv::Mat_<double>(3,3) <<
                                   5.1314616546024763e+02, 0., 3.2408680711198656e+02,
                                   0.,5.1307120574033013e+02, 2.5053770230445755e+02,
                                   0., 0., 1.);

//        MatchPoints tmp_match;

//        std::vector<MatchPoints> match1;

//        tmp_match.first2d = cv::Point2d(5,4);
//        tmp_match.second2d = cv::Point2d(5,4);
//        tmp_match.first3d = cv::Point3d(-1,0,7);
//        tmp_match.second3d = cv::Point3d(-1,0,7);
//        match1.push_back(tmp_match);

//        tmp_match.first2d = cv::Point2d(7,3);
//        tmp_match.second2d = cv::Point2d(7,3);
//        tmp_match.first3d = cv::Point3d(2,0,9);
//        tmp_match.second3d = cv::Point3d(2,0,9);
//        match1.push_back(tmp_match);

//        queueFrame.push_back(FrameData(match1,Triple()));

//        /////////////////////////
//        std::vector<MatchPoints> match2;

//        tmp_match.first2d = cv::Point2d(5,4);
//        tmp_match.second2d = cv::Point2d(4,5);
//        tmp_match.first3d = cv::Point3d(-1,0,7);
//        tmp_match.second3d = cv::Point3d(-2,0,9);
//        match2.push_back(tmp_match);

//        tmp_match.first2d = cv::Point2d(7,3);
//        tmp_match.second2d = cv::Point2d(6,5);
//        tmp_match.first3d = cv::Point3d(2,0,9);
//        tmp_match.second3d = cv::Point3d(3,0,8);
//        match2.push_back(tmp_match);

//        queueFrame.push_back(FrameData(match2,Triple()));

//        /////////////////////////////////
//        std::vector<MatchPoints> match3;

//        tmp_match.first2d = cv::Point2d(4,3);
//        tmp_match.second2d = cv::Point2d(1,1);
//        tmp_match.first3d = cv::Point3d(-2,0,11);
//        tmp_match.second3d = cv::Point3d(-1,0,10);
//        match3.push_back(tmp_match);

//        tmp_match.first2d = cv::Point2d(5,4);
//        tmp_match.second2d = cv::Point2d(1,2);
//        tmp_match.first3d = cv::Point3d(-2,0,9);
//        tmp_match.second3d = cv::Point3d(-3,0,8);
//        match3.push_back(tmp_match);

//        tmp_match.first2d = cv::Point2d(6,5);
//        tmp_match.second2d = cv::Point2d(3,4);
//        tmp_match.first3d = cv::Point3d(3,0,8);
//        tmp_match.second3d = cv::Point3d(0,0,6);
//        match3.push_back(tmp_match);

//        queueFrame.push_back(FrameData(match3,Triple()));
//        numCamera = 3;
    }

    void run();
    int pushFrame(std::vector<MatchPoints>, Triple);

private:
    struct FrameData
    {
        std::vector<MatchPoints> matches;
        Triple triple;
        FrameData(std::vector<MatchPoints> matches, Triple triple):
            matches(matches), triple(triple)
        {}
    };

    int numCamera;
    std::deque<FrameData> queueFrame;
    // camera related parameters

    cv::Size  cameraRes;
    cv::Mat   cameraMat;
};
}
#endif //OPENCV_BOUNDLE_ADJUSTMENT_H
