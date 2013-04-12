#ifndef RTFINDERTRILATERATION_H
#define RTFINDERTRILATERATION_H
#include "irtfinder.h"
#include "fstream"
namespace LuxSlam
{
    class RTFinderTrilateration : public IRTFinder
    {
        /// compute rotation matrix and translation vector from match points
        /**
            \param matches - match points
            \return structure fill 2d and 3d coordinates for match point
        */
        Triple getRTVector(const std::vector <MatchPoints>& matches);

        /// determining coordinats of point by measurement of distances and coordinats of three points
        /**
            \param triple - structure fill 2d and 3d coordinates for three points
            \param firstSolution - one of possible positions of point
            \param secondSolution - another possible postion of point
            \return 0 - if positions of points can not determine;
                    1 - if only one position of point determined and save in param firstSolution;
                    2 - if deramined two postions of points
        */
        int trilateration(Triple triple, cv::Point3d& firstSolution, cv::Point3d& secondSolution);

        /// select an optimal translation vector from set based on value mode
        /**
            \param various_vectors - set of vectors
            \return structure contain 2d and 3d coordinates optimal vector
        */
        Triple getOptimumVector(std::vector< Triple > various_vectors);

        /// compute rotation matrix between two space by three points from each space
        /**
            \param data - structure contatin 3d coordinated of points from each space
            \return matrix rotation
        */
        cv::Mat findMatrixRotation(const Triple data);
    public:

        /// sort match points by three in spacial structure Triple. Sorting use longest destances between each of three points
        /**
            \param points - points for sorting
            \return sorted points by three int structure Triple
        */
        std::vector< Triple > sortByTriple(std::vector<MatchPoints> points);

        RTFinderTrilateration();
    };
}
#endif // RTFINDERTRILATERATION_H
