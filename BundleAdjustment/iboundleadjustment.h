#ifndef IBOUNDLE_ADJUSTMENT_H
#define IBOUNDLE_ADJUSTMENT_H

#include "opencv2/contrib/contrib.hpp"
#include "matchpoints.h"
#include "vector"
#include "triples.h"
namespace LuxSlam
{
    class IBoundleAdjustment
    {

    public:
        /// run algorithm bundle adjustment
        virtual void run() = 0;

        /// add match points and coordinats of frame to processing in bundle adjustment
        /**
            \param matches - match points between images
            \param triple - 2d and 3d coordinats
            \return 1 - success
        */
        virtual int pushFrame(std::vector<MatchPoints> matches, Triple triple) = 0;
        IBoundleAdjustment(){}
        virtual ~IBoundleAdjustment(){}
    };
}
#endif //IBOUNDLE_ADJUSTMENT_H
