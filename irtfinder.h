#ifndef IRTFINDER_H
#define IRTFINDER_H
#include "triples.h"
#include "luxframe.hpp"
#include "matchpoints.h"
#include "staticfunctions.h"
namespace LuxSlam
{
    class IRTFinder
    {
    public:
        IRTFinder();
        virtual Triple getRTVector(const std::vector <MatchPoints>&) = 0;
    };
}
#endif // IRTFINDER_H
