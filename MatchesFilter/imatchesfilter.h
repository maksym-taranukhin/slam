#ifndef IMATCHESFILTER_H
#define IMATCHESFILTER_H
#include "vector"
#include "opencv2/features2d/features2d.hpp"
#include "luxframe.hpp"
#include "../FeaturesDetector/ifeaturedetector.h"
#include "matchpoints.h"
#include "../AdditionFunctions/staticfunctions.h"
namespace LuxSlam
{
    class IMatchesFilter
    {
        public:
        IMatchesFilter();

        /// filter the set of correspondences from false correspondences
        /**
            \param matches - correspondences set
            \return filtred correspondences set
        */
        virtual std::vector< MatchPoints >  filterMatches(
                std::vector <MatchPoints> matches) = 0;
    };
}
#endif // IMATCHESFILTER_H
