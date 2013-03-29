#ifndef OPENCV_BOUNDLE_ADJUSTMENT_H
#define OPENCV_BOUNDLE_ADJUSTMENT_H

#include "iboundleadjustment.h"
#include "queue"
#include "triples.h"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "staticfunctions.h"

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
    }

    void run();
    int pushFrame(std::vector<MatchPoints>, std::vector<Triple>);
private:
    struct FrameData
    {
        std::vector<MatchPoints> matches;
        std::vector<Triple> triples;
        FrameData(std::vector<MatchPoints> matches,std::vector<Triple> triples):
            matches(matches), triples(triples)
        {}
    };

    int numCamera;
    std::vector<FrameData> queueFrame;
    // camera related parameters

    cv::Size  cameraRes;
    cv::Mat   cameraMat;
};
}
#endif //OPENCV_BOUNDLE_ADJUSTMENT_H
