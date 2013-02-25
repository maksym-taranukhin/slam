#ifndef MATCHESFILTERCVTEAM_H
#define MATCHESFILTERCVTEAM_H
#include "imatchesfilter.h"

namespace LuxSlam
{
    class MatchesFilterCVTeam : public IMatchesFilter
    {
    public:
        MatchesFilterCVTeam();
        std::vector< MatchPoints > filterMatches(
                std::vector <MatchPoints> );
    };
}
#endif // MATCHESFILTERCVTEAM_H
