#include "matchesfiltercvteam.h"
namespace LuxSlam
{
    MatchesFilterCVTeam::MatchesFilterCVTeam()
    {
    }

    std::vector< MatchPoints > MatchesFilterCVTeam::filterMatches(
            std::vector <MatchPoints> input_matches)
    {
        std::vector <MatchPoints> good_matches;

        for( int i = 0; i < input_matches.size(); i++ )
        {
              cv::Point3d point3d1 = input_matches.at(i).first3d;
              cv::Point3d point3d2 = input_matches.at(i).second3d;

              if( (point3d1.z!=0) && (point3d2.z!=0) &&fabs(point3d1.z-point3d2.z)<1.0)
              {
                    good_matches.push_back( input_matches.at(i));
              }
        }


        std::vector< MatchPoints > very_good_matches;
        int min_failed=100000;
        int min_index=-1;

        for (int ii = 0; ii<good_matches.size(); ii+=1)
        {
                int failed=0;
                cv::Point3d curr_p3d = good_matches.at(ii).first3d;
                cv::Point3d curr_p3d2 = good_matches.at(ii).second3d;

                for( int i = 0; i < good_matches.size(); i++ )
                {
                      cv::Point3d p3d1 = good_matches.at(i).first3d;
                      cv::Point3d p3d2 = good_matches.at(i).second3d;

                      const float max_vec = 0.05;
                      float len1 = StaticFunctions::distance3d(curr_p3d,p3d1);
                      float len2 = StaticFunctions::distance3d(curr_p3d2,p3d2);

                      if( (fabs (len1-len2)) >max_vec)  failed+=1;

                }
                if (min_failed>failed)
                {
                    min_index=ii;
                    min_failed=failed;
                }

        }

        if (min_index>=0)
        {
            cv::Point3d curr_p3d = good_matches.at(min_index).first3d;
            cv::Point3d curr_p3d2 = good_matches.at(min_index).second3d;

            for( int i = 0; i < good_matches.size(); i++ )
            {
                  cv::Point3d p3d1 = good_matches.at(i).first3d;
                  cv::Point3d p3d2 = good_matches.at(i).second3d;

                  const float max_vec = 0.025;
                  float len1 = StaticFunctions::distance3d(curr_p3d,p3d1);
                  float len2 = StaticFunctions::distance3d(curr_p3d2,p3d2);

                  if( (fabs (len1-len2)) <max_vec)

                      if( fabs(curr_p3d.x-p3d1.x-(curr_p3d2.x-p3d2.x))<0.1
                                            && fabs(curr_p3d.y-p3d1.y-(curr_p3d2.y-p3d2.y))<0.1
                                            && fabs(curr_p3d.z-p3d1.z-(curr_p3d2.z-p3d2.z))<0.1
                                            )
                                            very_good_matches.push_back(good_matches[i]);
            }
        }
        qDebug()<<input_matches.size()<<very_good_matches.size()<<"before filter size";
        return very_good_matches;
       // return good_matches;
    }
}
