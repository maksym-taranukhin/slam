#ifndef IBOUNDLE_ADJUSTMENT_H
#define IBOUNDLE_ADJUSTMENT_H

#include "opencv2/contrib/contrib.hpp"
#include "matchpoints.h"
#include "vector"

namespace LuxSlam
{
    class IBoundleAdjustment
    {
    public:
        virtual void run() = 0;
        virtual int pushFrame(std::vector<MatchPoints>) = 0;
        IBoundleAdjustment();
        virtual ~IBoundleAdjustment(){}
    };
}
#endif //IBOUNDLE_ADJUSTMENT_H
