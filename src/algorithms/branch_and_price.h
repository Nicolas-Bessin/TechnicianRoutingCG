#pragma once

#include "instance/instance.h"
#include "routes/route.h"

#include <tuple>
#include <set>
#include <vector>

void branch_and_price(
    const Instance& instance, 
    std::vector<Route>& routes,
    int max_resources_dominance = -1,
    bool cyclic_pricing = true,
    int time_limit_per_node = 60,
    int max_depth = 10,
    bool verbose = true
    );


// Convert a vector of routes to the set of required edges it contains
std::set<std::tuple<int, int, int>> routes_to_required_edges(const std::vector<Route>& routes);