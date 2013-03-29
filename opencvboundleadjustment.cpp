#include "opencvboundleadjustment.h"
namespace LuxSlam
{

int OpenCVBoundleAdjustemnt::pushFrame(std::vector<MatchPoints> matchers, std::vector<Triple> triples)
{
    for (int i = 0; i < matchers.size(); ++i)
    {
        cv::Point3d tempPoint = matchers[i].second3d;
        matchers[i].second3d = StaticFunctions::transformPoint3d(tempPoint,
                                                                 triples[i].rotation_matrix,
                                                                 triples[i].translation_vector);
    }

    queueFrame.push_back(FrameData(matchers, triples));

    int size = queueFrame.size();

    if (size >= numCamera)
    {
        run();
        queueFrame.pop_back();
    }

    return 1;
}

void OpenCVBoundleAdjustemnt::run()
{

    // variables for sba
    std::vector<cv::Point3d>                points_true, points_init, points_opt;
    std::vector<std::vector<cv::Point2d> >  imagePoints;
    std::vector<std::vector<int> >          visiblity;
    std::vector<cv::Mat>                    cameraMatrix;
    std::vector<cv::Mat>                    R_true, R_init, R_opt;
    std::vector<cv::Mat>                    T_true, T_init, T_opt;
    std::vector<cv::Mat>                    distCoeffs;
    cv::TermCriteria                        criteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 70, 1e-10);

    for (unsigned int i = 0; i < queueFrame.size(); ++i)
        for (unsigned int j = 0; j < queueFrame.size(); ++j)
            points_opt.push_back(queueFrame[i].matches[i].second3d);


    // define cameras
    for (int i=0; i<numCamera; i++) {
      cameraMatrix.push_back(cameraMat);
      distCoeffs.push_back((cv::Mat_<double>(4,1) << 0, 0, 0, 0));
    }

    // project points to image coordinates
    for (int i=0; i<cameraMatrix.size(); i++)
    {
      //  std::vector<cv::Point2d>  imagePoint = queueFrame[i].matches[i].second2d;
    //    std::vector<int> vis(imagePoint.size(), 0);

     //   vis[]

    //  imagePoints.push_back(imagePoint);
    //  visiblity.push_back(vis);
    }

    // run bunble adjustment
    cv::LevMarqSparse   lms;
    lms.bundleAdjust(points_opt, imagePoints, visiblity, cameraMatrix, R_opt, T_opt, distCoeffs, criteria);
}
}
