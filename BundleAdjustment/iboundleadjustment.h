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
        virtual void run() = 0;
        virtual int pushFrame(std::vector<MatchPoints>, Triple) = 0;
        IBoundleAdjustment(){}
        virtual ~IBoundleAdjustment(){}
    };
}
#endif //IBOUNDLE_ADJUSTMENT_H
