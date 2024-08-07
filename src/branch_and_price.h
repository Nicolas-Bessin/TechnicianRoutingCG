#pragma once

#include "instance.h"
#include "solution.h"

#include "master.h"

#include <vector>
#include <tuple>
#include <set>

struct BPNode {
    // Node depth in the branch and price tree
    int depth;
    // Node upper bound and lower bound
    double upper_bound;
    double lower_bound;
    // Active routes in the node : a route is active if its coefficient is 1
    std::vector<int> active_routes;
    // Upper bounds constraints imposed on the x_ijv variables - x_ijv <= 0 for (i, j, v) in upper_bounds
    std::set<std::tuple<int, int, int>> upper_bounds;
    // Lower bounds constraints imposed on the x_ijv variables - x_ijv >= 1 for (i, j, v) in lower_bounds
    std::set<std::tuple<int, int, int>> lower_bounds;
};