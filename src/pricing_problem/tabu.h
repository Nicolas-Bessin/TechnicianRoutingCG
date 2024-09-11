#pragma once

#include "master_problem/master.h"
#include "routes/route.h"

#include <vector>

std::vector<Route> tabu_search(
    const Route & initial_route,
    int max_iterations,
    int max_modifications,
    const DualSolution & solution,
    const Vehicle & vehicle,
    const Instance & instance
);