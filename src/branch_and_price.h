#pragma once

#include "instance.h"
#include "route.h"

#include <tuple>
#include <set>
#include <vector>

struct BPNode {
    // Node depth in the branch and price tree
    int depth;
    // Node upper bound - best value we could get from this node
    double upper_bound;
    // Node lower bound - best overall solution we have found so far
    double lower_bound;
    // Active routes in the node : for r in active_routes, routes[r] is active
    std::set<int> active_routes;
    // Upper bounds constraints imposed on the x_ijv variables - x_ijv <= 0 for (i, j, v) in upper_bounds
    std::set<std::tuple<int, int, int>> upper_bound_cuts;
    // Lower bounds constraints imposed on the x_ijv variables - x_ijv >= 1 for (i, j, v) in lower_bounds
    std::set<std::tuple<int, int, int>> lower_bound_cuts;

};

// Create the root node of the branch and price tree from a set of initial routes
BPNode RootNode(const std::vector<Route>& initial_routes);

void branch_and_price(
    const Instance& instance, 
    const std::vector<Route>& initial_routes,
    double reduced_cost_threshold,
    int time_limit_per_node = 60,
    int max_depth = 10
    );


// Convert a vector of routes to the set of required edges it contains
std::set<std::tuple<int, int, int>> routes_to_required_edges(const std::vector<Route>& routes);