#include "../opencvboundleadjustment.h"
#include "iostream"
#include "Logger.hpp"
namespace LuxSlam
{

int OpenCVBoundleAdjustemnt::pushFrame(std::vector<MatchPoints> matches, Triple triple)
{

    // transform all frame 3d points into first coordinate system
    int temp_size = matches.size();
    for (int i = 0; i < temp_size; ++i)
    {
        cv::Point3d tempPoint = matches[i].second3d;
        matches[i].second3d = StaticFunctions::transformPoint3d(tempPoint,
                                                                 triple.rotation_matrix,
                                                                 triple.translation_vector);
    }

    queueFrame.push_back(FrameData(matches, triple));

    // if frames count >= needed, then run algorithm
    temp_size = queueFrame.size();

    if (temp_size >= numCamera)
    {
        run();
        queueFrame.pop_front();
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

    // define cameras
    for (int i=0; i<numCamera; i++)
    {
      cameraMatrix.push_back(cameraMat);
      distCoeffs.push_back((cv::Mat_<double>(4,1) << 0, 0, 0, 0));
    }


    for (unsigned int i = 0; i < cameraMatrix.size(); ++i)
        for (unsigned int j = 0; j < queueFrame[i].matches.size(); ++j)
            points_opt.push_back(queueFrame[i].matches[j].second3d);


    for (int i=0 ; i < cameraMatrix.size(); i++)
    {
        imagePoints.push_back(std::vector <cv::Point2d>());
        visiblity.push_back(std::vector<int>());
    }

    for (int i=0 ; i < cameraMatrix.size(); i++)
    {
        std::vector <MatchPoints> matches = queueFrame[i].matches;
        for (int j = 0 ; j< matches.size(); j++)
        {
            for (int k = 0 ; k < cameraMatrix.size() ; k++)
            {
                if (i == k)
                {
                    imagePoints.at(k).push_back(matches.at(j).second2d);
                    visiblity.at(k).push_back(1);
                }
                else if (i == k+1)
                {
                    imagePoints.at(k).push_back(matches.at(j).first2d);
                    visiblity.at(k).push_back(1);

                }
                else
                {
                    imagePoints.at(k).push_back(cv::Point2d(-1,-1));
                    visiblity.at(k).push_back(0);
                }
            }
        }
    }

    for (int i = 0 ; i<imagePoints.size();i++)
    {
        for (int j = 0 ; j<imagePoints.at(i).size(); j++)
            std::cerr<<imagePoints.at(i).at(j)<<" ";
        std::cerr<<"\n";
    }
    for (int i = 0 ; i<visiblity.size();i++)
    {
        for (int j = 0 ; j<visiblity.at(i).size(); j++)
            std::cerr<<visiblity.at(i).at(j)<<" ";
        std::cerr<<"\n";
    }

    for (int i= 0;i<cameraMatrix.size();i++)
    {
        R_opt.push_back(queueFrame.at(i).triple.rotation_matrix);

        T_opt.push_back(cv::Mat(queueFrame.at(i).triple.translation_vector));


    }
    // run bunble adjustment
    cv::LevMarqSparse   lms;
    lms.bundleAdjust(points_opt, imagePoints, visiblity, cameraMatrix, R_opt, T_opt, distCoeffs, criteria);

    cv::Mat eulerAngles = StaticFunctions::getEulerAngles(R_opt.at(0));

    Logger & l = Logger::getInstance ();

    l.logCamera(cv::Point3d(T_opt.at(0)),eulerAngles.at<float>(0)/M_PI*180,eulerAngles.at<float>(1)/M_PI*180,eulerAngles.at<float>(2)/M_PI*180);

}
}
