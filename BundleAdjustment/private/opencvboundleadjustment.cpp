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
        std::vector<cv::Point3d>                points_opt;
        std::vector<std::vector<cv::Point2d> >  imagePoints;
        std::vector<std::vector<int> >          visiblity;
        std::vector<cv::Mat>                    cameraMatrix;
        std::vector<cv::Mat>                    R_opt;
        std::vector<cv::Mat>                    T_opt;
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
            std::vector<int>  vis(points_opt.size(), 0);
            std::vector<cv::Point2d>  imgp(points_opt.size());

            int counter = 0 ;

            for (int j = 0; j<queueFrame.size() ; j++)
            {
                std::vector<MatchPoints> matches = queueFrame.at(j).matches;

                for (int k = 0 ; k<matches.size(); k++)
                {
                    if (j == i)
                    {
                        vis.at(counter) = 1;
                        imgp.at(counter) = matches.at(k).second2d;
                    }

                    else
                    {
                        vis.at(counter) = 0;
                        imgp.at(counter) = cv::Point2d(-1,-1);
                    }
                    counter++;
                }
            }

            counter = 0 ;

            for (int j = 0; j<queueFrame.size() ; j++)
            {
                std::vector<MatchPoints> matches = queueFrame.at(j).matches;

                for (int k = 0 ; k<matches.size(); k++)
                {
                    if (j == i+1 && j>0)
                    {
                        vis.at(counter) = 1;
                        imgp.at(counter) = matches.at(k).first2d;
                    }
                    counter++;
                }
            }

            visiblity.push_back(vis);
            imagePoints.push_back(imgp);
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
        for (int i = 0; i < points_opt.size(); ++i)
            std::cerr <<"("<< points_opt[i].x << ", " << points_opt[i].y << ", "<< points_opt[i].z << ")\n";

        std::cerr <<"tvec"<<T_opt.at(1)<<"\n";
        // run bunble adjustment
        cv::LevMarqSparse   lms;
        lms.bundleAdjust(points_opt, imagePoints, visiblity, cameraMatrix, R_opt, T_opt, distCoeffs, criteria);
        std::cerr <<"tvec"<<T_opt.at(1)<<"\n";

        for (int i = 0; i < points_opt.size(); ++i)
            std::cerr <<"("<< points_opt[i].x << ", " << points_opt[i].y << ", "<< points_opt[i].z << ")\n";

        cv::Mat eulerAngles = StaticFunctions::getEulerAngles(R_opt.at(1));

        arstudio::Logger & l = arstudio::Logger::instance();

        l.log_camera(cv::Point3d(T_opt.at(1)),eulerAngles.at<float>(0)/M_PI*180,eulerAngles.at<float>(1)/M_PI*180,eulerAngles.at<float>(2)/M_PI*180,"BA-camera");

    }
}
