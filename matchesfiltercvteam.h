#ifndef MATCHESFILTERCVTEAM_H
#define MATCHESFILTERCVTEAM_H
#include "imatchesfilter.h"

namespace LuxSlam
{
    class MatchesFilterCVTeam : public IMatchesFilter
    {
    public:
        MatchesFilterCVTeam();
        // points of matches must not have the point.z == 0; (undefined points)
        std::vector< MatchPoints > filterMatches(
                std::vector <MatchPoints> );
    };
}
#endif // MATCHESFILTERCVTEAM_H
