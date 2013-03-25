#ifndef RTFINDERTRILATERATION_H
#define RTFINDERTRILATERATION_H
#include "irtfinder.h"
#include "fstream"
namespace LuxSlam
{

    class RTFinderTrilateration : public IRTFinder
    {
        Triple getRTVector(const std::vector <MatchPoints>&);
        int trilateration(Triple triple, cv::Point3d& firstSolution, cv::Point3d& secondSolution);
        Triple getOptimumVector(std::vector< Triple > various_vectors);
        cv::Mat findMatrixRotation(const Triple data);
    public:
          std::vector< Triple > sortByTriple(std::vector<MatchPoints> points);

        RTFinderTrilateration();
    };
}
#endif // RTFINDERTRILATERATION_H
