#pragma once

#include <tuple>
#include <set>

struct BPNode {
    // Node depth in the branch and price tree
    int depth;
    // Node upper bound and lower bound
    double upper_bound;
    double lower_bound;
    // Active routes in the node : for r in active_routes, routes[r] is active
    std::set<int> active_routes;
    // Upper bounds constraints imposed on the x_ijv variables - x_ijv <= 0 for (i, j, v) in upper_bounds
    std::set<std::tuple<int, int, int>> upper_bounds;
    // Lower bounds constraints imposed on the x_ijv variables - x_ijv >= 1 for (i, j, v) in lower_bounds
    std::set<std::tuple<int, int, int>> lower_bounds;
};


void branch_and_price(
    const Instance& instance, 
    const std::vector<Route>& initial_routes,
    double reduced_cost_threshold,
    int time_limit_per_node = 60,
    int max_depth = 10,
    bool compute_integer_solution = true,
    bool verbose = true
    );