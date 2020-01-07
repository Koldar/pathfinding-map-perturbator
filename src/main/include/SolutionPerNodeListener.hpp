#ifndef _PATHFINDINGMAPPERTURBATOR_SOLUTIONPERNODELISTENER_HEADER__
#define _PATHFINDINGMAPPERTURBATOR_SOLUTIONPERNODELISTENER_HEADER__

#include <pathfinding-utils/AstarAllSolutionsListener.hpp>

namespace pathfinding::map_perturbator::utils {

    using pathfinding;

    template <typename STATE>
    class SolutionPerNodeListener: public AstarAllSolutionsListener<STATE> {

    }

}

#endif