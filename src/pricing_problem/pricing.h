#pragma once


#include "instance/instance.h"
#include "routes/route.h"
#include "master_problem/master.h"

#include <vector>

std::vector<Route> solve_pricing_problems_basic(
    const MasterSolution & solution,
    const Instance & instance,
    bool using_cyclic_pricing,
    int n_ressources_dominance,
    double reduced_cost_threshold = 1e-6
);