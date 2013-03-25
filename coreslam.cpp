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

        // get keypoints and descriptors
        curr_features = fdetector->getFeatures(curr_frame);

        if (curr_features->points.size() == 0)
        {
            delete curr_features;
            return;
        }

        // drawing results
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

            // drawing results
            for (int i=0;i<points.size();i++)
                cv::line(results,points.at(i).first2d,points.at(i).second2d,cvScalar(0,190),3);

            // getting rotation, translation vector
            result = rtfinder->getRTVector(points);

            cv::Mat eulerAngles = StaticFunctions::getEulerAngles(result.rotation_matrix);

            std::cerr<<"before\n"<<eulerAngles<<"\n";
            result.rotation_matrix = StaticFunctions::getRotationMatrix(eulerAngles);

            std::cerr<<"after\n"<<StaticFunctions::getEulerAngles(result.rotation_matrix)<<"\n";


            std::cerr << "vector: " << result.translation_vector << "\n";
            std::cerr << "distance: " << StaticFunctions::distance3d(cv::Point3d(0,0,0), result.translation_vector) << "\n";
            std::cerr << global_transformation_vector.rotation_matrix << "\n";

            cv::Point3d not_rotated_vector = result.translation_vector;

            result.translation_vector.x=
                    global_transformation_vector.rotation_matrix.at<float>(0,0)*not_rotated_vector.x+
                    global_transformation_vector.rotation_matrix.at<float>(0,1)*not_rotated_vector.y+
                    global_transformation_vector.rotation_matrix.at<float>(0,2)*not_rotated_vector.z;

            result.translation_vector.y=
                    global_transformation_vector.rotation_matrix.at<float>(1,0)*not_rotated_vector.x+
                    global_transformation_vector.rotation_matrix.at<float>(1,1)*not_rotated_vector.y+
                    global_transformation_vector.rotation_matrix.at<float>(1,2)*not_rotated_vector.z;

            result.translation_vector.z=
                    global_transformation_vector.rotation_matrix.at<float>(2,0)*not_rotated_vector.x+
                    global_transformation_vector.rotation_matrix.at<float>(2,1)*not_rotated_vector.y+
                    global_transformation_vector.rotation_matrix.at<float>(2,2)*not_rotated_vector.z;

            std::cerr << "after vector: " << result.translation_vector << "\n";
            std::cerr << "after distance: " << StaticFunctions::distance3d(cv::Point3d(0,0,0), result.translation_vector)<<"\n\n";

            // calculate the global rotation matrix and the translation vector
            global_transformation_vector.rotation_matrix =
                    global_transformation_vector.rotation_matrix * result.rotation_matrix;

            global_transformation_vector.translation_vector =
                    global_transformation_vector.translation_vector + result.translation_vector;
        }

        // Logging the camera

        cv::Mat eulerAngles = StaticFunctions::getEulerAngles(global_transformation_vector.rotation_matrix);
//        static int iterat = 0;
//        iterat ++;
//        cv::Mat tmp_rot = StaticFunctions::GenerateRotationMatrix(M_PI_2/iterat,1)*StaticFunctions::GenerateRotationMatrix(M_PI_2/iterat,2);
//        std::cerr<<"\n before\n"<<tmp_rot;
//        cv::Mat tmp_eul = StaticFunctions::getEulerAngles(tmp_rot);
//        tmp_rot = StaticFunctions::getRotationMatrix(tmp_eul);
//        std::cerr<<"\n after\n"<<tmp_rot<<"\n";


        Logger & l = Logger::getInstance ();

        l.logCamera(global_transformation_vector.translation_vector,eulerAngles.at<float>(0)/M_PI*180,eulerAngles.at<float>(1)/M_PI*180,eulerAngles.at<float>(2)/M_PI*180);
        l.addImage(results,"fatures");

        delete prev_frame;
        delete prev_features;

        prev_frame = new LuxFrame(*curr_frame);
        prev_features = curr_features;
    }
}
