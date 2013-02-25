#ifndef TRIPLES_H
#define TRIPLES_H
#include "opencv2/highgui/highgui.hpp"
#include "matchpoints.h"
namespace LuxSlam
{
    class Triple
    {
    public:
        MatchPoints first_point;
        MatchPoints second_point;
        MatchPoints third_point;
        cv::Point3d translation_vector;
        cv::Mat rotation_matrix;
        Triple()
        {
            translation_vector = cv::Point3d(0,0,0);
            rotation_matrix = cv::Mat(3,3,CV_32FC1);
        }

        cv::Point3d getPoint3d(int frame,int index) const
        {
            if (frame==0)
            {
                switch (index){
                case 0: return first_point.first3d;
                case 1: return second_point.first3d;
                case 2: return third_point.first3d;}
            }else
            {
                switch (index){
                case 0: return first_point.second3d;
                case 1: return second_point.second3d;
                case 2: return third_point.second3d;}
            }
        }
    };
}
#endif // TRIPLES_H
