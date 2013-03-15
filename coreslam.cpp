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

            // getting rotation, translation vector
            result = rtfinder->getRTVector(points);

            std::cerr<<"\ncurrent transl before rot Vec\n"<< result.translation_vector;
            std::cerr<<"\global rot matrice \n"<< global_rotation_translation_vector.rotation_matrix;

            cv::Point3d not_rotated_vector = result.translation_vector;
            result.translation_vector.x=
                    global_rotation_translation_vector.rotation_matrix.at<float>(0,0)*not_rotated_vector.x+
                    global_rotation_translation_vector.rotation_matrix.at<float>(0,1)*not_rotated_vector.y+
                    global_rotation_translation_vector.rotation_matrix.at<float>(0,2)*not_rotated_vector.z;

            result.translation_vector.y=
                    global_rotation_translation_vector.rotation_matrix.at<float>(1,0)*not_rotated_vector.x+
                    global_rotation_translation_vector.rotation_matrix.at<float>(1,1)*not_rotated_vector.y+
                    global_rotation_translation_vector.rotation_matrix.at<float>(1,2)*not_rotated_vector.z;


            result.translation_vector.z=
                    global_rotation_translation_vector.rotation_matrix.at<float>(2,0)*not_rotated_vector.x+
                    global_rotation_translation_vector.rotation_matrix.at<float>(2,1)*not_rotated_vector.y+
                    global_rotation_translation_vector.rotation_matrix.at<float>(2,2)*not_rotated_vector.z;

            std::cerr<<"\ncurrent transl Vec\n"<< result.translation_vector;
            // transform the global rotation, translation vector
            global_rotation_translation_vector.rotation_matrix =
                    global_rotation_translation_vector.rotation_matrix * result.rotation_matrix;

            global_rotation_translation_vector.translation_vector =
                    global_rotation_translation_vector.translation_vector + result.translation_vector;


            std::cerr<<"\nglobal transl Vec\n"<< global_rotation_translation_vector.translation_vector;

            }
        // Logging the camera

        cv::Mat eulerAngles = StaticFunctions::getEulerAngles(global_rotation_translation_vector.rotation_matrix);

        std::cerr <<"\n Angles\n" << eulerAngles/M_PI*180 << "\n";

        Logger & l = Logger::getInstance ();

        l.logCamera(global_rotation_translation_vector.translation_vector,eulerAngles.at<float>(0)/M_PI*180,eulerAngles.at<float>(1)/M_PI*180,eulerAngles.at<float>(2)/M_PI*180);
        l.addImage(results,"fatures");

        delete prev_frame;
        delete prev_features;

        prev_frame = new LuxFrame(*curr_frame);
        prev_features = curr_features;

//        return result;
    }
}
