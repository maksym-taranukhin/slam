#include "coreslam.h"
#include "QDebug"
#include "math.h"
#include <logger.hpp>

namespace LuxSlam
{
    CoreSlam::CoreSlam()
    {
        fdetector = new FeatureDetectorSurf();
        fmatcher = new FeatureMatcherFlann();
        rtfinder = new RTFinderTrilateration();
        ffilter = new MatchesFilterCVTeam();
        prev_frame = 0;
        prev_features = 0;
    }

    void CoreSlam::Get3dPointsOfMatches(const std::vector<cv::DMatch>& matches,  std::vector<MatchPoints>& points)
    {
        //todo in function
        for (unsigned int i = 0 ; i< matches.size(); i++)
        {
            cv::KeyPoint p1 = prev_features->points.at(matches.at(i).queryIdx);
            cv::KeyPoint p2 = curr_features->points.at(matches.at(i).trainIdx);
            MatchPoints mp;
            mp.first2d = p1.pt;
            mp.second2d = p2.pt;

            mp.first3d = prev_frame->getPoint3D(mp.first2d.x,mp.first2d.y);
            mp.second3d = curr_frame->getPoint3D(mp.second2d.x,mp.second2d.y);

            if (mp.first3d.z && mp.second3d.z) points.push_back(mp);
        }
    }

    void CoreSlam::run(LuxFrame * input_frame)
    {
        // algorithm
        curr_frame = input_frame;
        Triple result;

        curr_features = fdetector->getFeatures(curr_frame);// get keypoints and descriptors

        if (curr_features->points.size() == 0)
        {
            delete curr_features;
            return;
        }

        ////Drawing results

        cv::Mat results;
        curr_frame->image.copyTo(results);
        for (int i=0; i<curr_features->points.size();i++)
            cv::circle(results,curr_features->points.at(i).pt,5,cvScalar(100),3);



        if (prev_frame)  // if this is the first frame
        {
            // matching
            std::vector< cv::DMatch > matches = fmatcher->getMatches(prev_features,curr_features);

            // getting 3d coordinates of matches
            std::vector <MatchPoints> points;

            Get3dPointsOfMatches(matches, points);

            // filtering the matches
            points = ffilter->filterMatches(points);

            //Drawing results
            for (int i=0;i<points.size();i++)
                cv::line(results,points.at(i).first2d,points.at(i).second2d,cvScalar(0,190),3);

            result = rtfinder->getRTVector(points);



           // std::ofstream file("/home/max/config.txt");
           // file<<"2\n";
           // file<<"0 0 0 0 0 0\n";
            //file<<"0 0 0 0 0 0\n";

            result.translation_vector.x *= (-1);
            result.translation_vector.y *= (-1);
            result.translation_vector.z *= (-1);


            cv::Mat inverted_rotation_matrix(3, 3, CV_32FC1);
            cv::invert(result.rotation_matrix,inverted_rotation_matrix);

            //curr_frame->Transform(result.rotation_matrix, result.translation_vector);

            global_rotation_translation_vector.translation_vector =
                    global_rotation_translation_vector.translation_vector + result.translation_vector;

            global_rotation_translation_vector.rotation_matrix *= inverted_rotation_matrix;

            cv::Mat rodrigues(3, 1, CV_32FC1);
            cv::Rodrigues(global_rotation_translation_vector.rotation_matrix,rodrigues);

            Logger & l = Logger::getInstance ();

            l.logCamera(global_rotation_translation_vector.translation_vector,-(rodrigues.at<float>(0)/3.14*180),-(rodrigues.at<float>(1)/3.14*180),-(rodrigues.at<float>(2))/3.14*180);
            l.addImage(results,"fatures");
//            for (int i = 0 ; i < 3 ; i++)
//                file <<-rodrigues.at<float>(i)/M_PI*(float)180<<" ";
/*
            file <<result.translation_vector.x<<" ";
            file <<result.translation_vector.y<<" ";
            file <<result.translation_vector.z<<" ";
            file.close();

            file.open("/home/max/ny0.txt");
             for (int i=0;i<640;i+=3)
                 for (int j=0;j<480;j+=3)
                 {
                     file<<(prev_frame->getPoint3D(i,j).x)<<" ";
                     file<<prev_frame->getPoint3D(i,j).y<<" ";
                     file<<prev_frame->getPoint3D(i,j).z<<" ";
                     file<<prev_frame->image.data[(j*640+i)*3]/255.0<<" ";
                     file<<prev_frame->image.data[(j*640+i)*3+1]/255.0<<" ";
                     file<<prev_frame->image.data[(j*640+i)*3+2]/255.0<<"\n";
                 }
             file.close();
             file.open("/home/max/ny1.txt");
              for (int i=0;i<640;i+=3)
                  for (int j=0;j<480;j+=3)
                  {
                      //curr_frame->ExecuteTransformation();
                      file<<curr_frame->getPoint3D(i,j).x<<" ";
                      file<<curr_frame->getPoint3D(i,j).y<<" ";
                      file<<curr_frame->getPoint3D(i,j).z<<" ";
                      file<<curr_frame->image.data[(j*640+i)*3]/255.0<<" ";
                      file<<curr_frame->image.data[(j*640+i)*3+1]/255.0<<" ";
                      file<<curr_frame->image.data[(j*640+i)*3+2]/255.0<<"\n";
                  }
              file.close();
*/
        }

        // ********dbg
        cv::imshow("surf",results);

        delete prev_frame;
        delete prev_features;

        prev_frame = new LuxFrame(*curr_frame);
        qDebug()<<prev_frame->depth_map.data<<curr_frame->depth_map.data;
        prev_features = curr_features;

//        return result;
    }
}
