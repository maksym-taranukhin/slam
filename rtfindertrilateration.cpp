#include "rtfindertrilateration.h"
#include "staticfunctions.h"
#include <errno.h>
#include <iostream>
#include <opencv/cv.h>

namespace LuxSlam
{
    cv::Point3d operator-(const cv::Point3d& a,const cv::Point3d& b)
    {
        cv::Point3d temp;
        temp.x = a.x - b.x;
        temp.y = a.y - b.y;
        temp.z = a.z - b.z;
        return temp;
    }

    cv::Point3d operator+(const cv::Point3d& a,const cv::Point3d& b)
    {
        cv::Point3d temp;
        temp.x = a.x + b.x;
        temp.y = a.y + b.y;
        temp.z = a.z + b.z;
        return temp;
    }


    RTFinderTrilateration::RTFinderTrilateration()
    {
    }
    int RTFinderTrilateration::trilateration(Triple triple, cv::Point3d& firstSolution,
                                             cv::Point3d& secondSolution)
    {
        cv::Point3d e_x = triple.second_point.first3d - triple.first_point.first3d;
        double normaEx = cv::norm(e_x);
        e_x = e_x * (1/normaEx);

        double distants[]={
            StaticFunctions::distance3d(cv::Point3d(0,0,0), triple.first_point.second3d),
            StaticFunctions::distance3d(cv::Point3d(0,0,0), triple.second_point.second3d),
            StaticFunctions::distance3d(cv::Point3d(0,0,0), triple.third_point.second3d),
        };

        if(distants[1]<(normaEx - distants[0]) || distants[1]>(normaEx + distants[0]))
        {
            //qDebug() << "Trilateration: Erorr in r2";
            return 0;
        }

        firstSolution = triple.third_point.first3d - triple.first_point.first3d;
        double i = e_x.dot(firstSolution);
        cv::Point3d e_y = firstSolution - (e_x*i);

        double normaEy = cv::norm(e_y);
        e_y = e_y * (1/normaEy);

        cv::Point3d e_z = e_x.cross(e_y);

        double j = e_y.dot(firstSolution);

        double r1sqv = distants[0]*distants[0];
        double r2sqv = distants[1]*distants[1];
        double r3sqv = distants[2]*distants[2];

        double x = (r1sqv - r2sqv + normaEx*normaEx)/(2*normaEx);
        double y = (r1sqv - r3sqv + (i*i) + (j*j))/(2*j) - (i/j)*x;
        errno = 0;
        double z = std::sqrt(r1sqv-x*x-y*y);

        if (errno == EDOM)
        {
            //qDebug() << "Trilateration: Erorr in sqrt";
            return 0;
        }

        e_x = e_x*x;
        e_y = e_y*y;
        e_z = e_z*z;

        firstSolution = triple.first_point.first3d + e_x + e_y + e_z;

        if (z > 0)
        {
            secondSolution = triple.first_point.first3d + e_x + e_y - e_z;
            return 2;
        } else return 1;
    }

    Triple RTFinderTrilateration::getOptimumVector(std::vector< Triple > various_vectors)
    {
        double summ = 0, min_summ = 0;
        Triple optimum;

        for (int i =0 ; i < various_vectors.size(); ++i)
            min_summ += StaticFunctions::distance3d(various_vectors.at(0).translation_vector,
                                               various_vectors.at(i).translation_vector);

        for (int i = 0; i < various_vectors.size(); ++i)
        {
            summ = 0;
            Triple curr_vector = various_vectors.at(i);
            for (int j =0; j < various_vectors.size(); ++j)
            {
                summ += StaticFunctions::distance3d(curr_vector.translation_vector,
                                                   various_vectors.at(j).translation_vector);
            }

            if (summ <= min_summ)
            {
                min_summ = summ;
                optimum = curr_vector;
            }
        }

        return optimum;
    }

    cv::Mat RTFinderTrilateration::findMatrixRotation(const Triple data)
    {
            cv::Mat b(3, 3, CV_32FC1);
            cv::Mat r(3, 3, CV_32FC1);
            cv::Mat A(3, 3, CV_32FC1);

            for (int i = 0;i<3 ; i++)
            {
                A.at<float>(i,0) = data.getPoint3d(0,i).x;
                A.at<float>(i,1) = data.getPoint3d(0,i).y;
                A.at<float>(i,2) = data.getPoint3d(0,i).z;
            }

            for (int i = 0;i<3 ; i++)
            {
                b.at<float>(i,0) = data.getPoint3d(1,i).x - data.translation_vector.x;
                b.at<float>(i,1) = data.getPoint3d(1,i).y - data.translation_vector.y;
                b.at<float>(i,2) = data.getPoint3d(1,i).z - data.translation_vector.z;
            }

            for (int i = 0; i < 3; ++i)
                    cv::solve(A, b.col(i), r.col(i), cv::DECOMP_SVD);

            return r;
    }

    Triple RTFinderTrilateration::getRTVector(const std::vector<MatchPoints> & points)
    {
        std::vector< Triple > triples = sortByTriple(points);
        std::vector< Triple > triples_with_translation_vector;
        Triple current_triple;
        cv::Point3d firstSolution, secondSolution;

        for (int triple_i = 0; triple_i < triples.size(); ++triple_i)
        {
            current_triple = triples.at(triple_i);

//            current_triple.first_point.first3d.x = 1;
//            current_triple.first_point.first3d.y = 1;
//            current_triple.first_point.first3d.z = 1;

//            current_triple.second_point.first3d.x = 1;
//            current_triple.second_point.first3d.y = -1;
//            current_triple.second_point.first3d.z = 1;

//            current_triple.third_point.first3d.x = -1;
//            current_triple.third_point.first3d.y = -1;
//            current_triple.third_point.first3d.z = 1;

//            current_triple.first_point.second3d.x = 1+10;
//            current_triple.first_point.second3d.y = 1-5;
//            current_triple.first_point.second3d.z = 1+5;

//            current_triple.second_point.second3d.x = 1+10;
//            current_triple.second_point.second3d.y = -1-5;
//            current_triple.second_point.second3d.z = 1+5;

//            current_triple.third_point.second3d.x = -1+10;
//            current_triple.third_point.second3d.y = -1-5;
//            current_triple.third_point.second3d.z = 1+5;

            int result = trilateration(current_triple, firstSolution, secondSolution);
           // qDebug()<<firstSolution.x*100<<firstSolution.y*100<<firstSolution.z*100;
             //   qDebug()<<secondSolution.x*100<<secondSolution.y*100<<secondSolution.z*100;
            if (result == 0) continue;
            else if (result == 2)
            {
                double first_distance = StaticFunctions::distance3d(cv::Point3d(0,0,0), firstSolution);
                double second_distance = StaticFunctions::distance3d(cv::Point3d(0,0,0), secondSolution);
                if (first_distance > second_distance) firstSolution = secondSolution;
            }

            current_triple.translation_vector = firstSolution;

            triples_with_translation_vector.push_back(current_triple);
        }

        Triple optimum_vector = getOptimumVector(triples_with_translation_vector);

//        optimum_vector.translation_vector.x = 0;
//        optimum_vector.translation_vector.y = 0;
//        optimum_vector.translation_vector.z = 0;
//        optimum_vector.rotation_matrix = StaticFunctions::getRotationMatrix(M_PI_2,3)*(StaticFunctions::getRotationMatrix(M_PI_2,2)
//                *StaticFunctions::getRotationMatrix(M_PI_2,1));
//        std::cerr << "rotmat: " << optimum_vector.rotation_matrix << "\n";
//        cv::transpose(findMatrixRotation(optimum_vector), optimum_vector.rotation_matrix);

        optimum_vector.rotation_matrix = findMatrixRotation(optimum_vector);


//        static int iterator = 0;
//        iterator ++;
//        switch (iterator)
//        {
//            case 1:
//                optimum_vector.translation_vector = cv::Point3d(0,0,2);
//                optimum_vector.rotation_matrix = StaticFunctions::getRotationMatrix(M_PI_2, 2);
//                break;
//            case 2:
//                optimum_vector.translation_vector = cv::Point3d(0,0,2);
//                optimum_vector.rotation_matrix = StaticFunctions::getRotationMatrix(-M_PI_2, 2);
//                break;
//            default:
//                optimum_vector.translation_vector = cv::Point3d(0,0,0);
//                optimum_vector.rotation_matrix = StaticFunctions::getRotationMatrix(0, 2);
//            break;
//        }
        return optimum_vector;
    }


    std::vector< Triple > RTFinderTrilateration::sortByTriple(std::vector<MatchPoints> points)
    {
        std::vector<MatchPoints> temp = points;
        std::vector<Triple> triples;

        int size = temp.size();

        Triple temp_triple;
        int num;
        for (int i = 0; i < size/3; ++i)
        {
            if (temp.at(i).first2d.x == -1) continue;
            temp_triple.first_point = temp.at(i);
            temp.at(i).first2d.x = -1;

            double maxDist = 0;
            int num;
            for (int j = 0; j < size; ++j)
            {
                if (temp.at(j).first2d.x != -1)
                    if (StaticFunctions::distance2d(temp_triple.first_point.first2d, temp.at(j).first2d) > maxDist)
                    {
                        maxDist = StaticFunctions::distance2d(temp_triple.first_point.first2d, temp.at(j).first2d);
                        temp_triple.second_point = temp.at(j);
                        num = j;
                    }
            }


            temp.at(num).first2d.x = -1;
            maxDist = 0;
            for (int j = 0; j < size; ++j)
            {
                if (temp.at(j).first2d.x != -1)
                    if ( std::fabs(StaticFunctions::distance2d(temp_triple.first_point.first2d, temp.at(j).first2d)) +
                         std::fabs(StaticFunctions::distance2d(temp.at(j).first2d, temp_triple.second_point.first2d)) > maxDist)
                    {
                        maxDist = std::fabs(StaticFunctions::distance2d(temp_triple.first_point.first2d, temp.at(j).first2d)) +
                                std::fabs(StaticFunctions::distance2d(temp.at(j).first2d, temp_triple.second_point.first2d));
                        temp_triple.third_point = temp.at(j);
                        num = j;
                    }
            }

            temp.at(num).first2d.x = -1;

            triples.push_back(temp_triple);
        }

        return triples;
    }
}
