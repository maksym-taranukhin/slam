#ifndef IMATCHESFILTER_H
#define IMATCHESFILTER_H
#include "vector"
#include "opencv2/features2d/features2d.hpp"
#include "luxframe.hpp"
#include "ifeaturedetector.h"
#include "matchpoints.h"
#include "staticfunctions.h"
namespace LuxSlam
{
    class IMatchesFilter
    {
        public:
        IMatchesFilter();
        virtual std::vector< MatchPoints >  filterMatches(
                std::vector <MatchPoints> ) = 0;
    };
}
#endif // IMATCHESFILTER_H
