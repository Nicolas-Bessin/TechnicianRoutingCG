#pragma once

#include "instance/instance.h"
#include "routes/route.h"
#include "algorithms/parameters.h"

#include <tuple>
#include <set>
#include <vector>

void branch_and_price(
    const Instance& instance, 
    std::vector<Route>& routes,
    const BranchAndPriceParameters& parameters
    );


// Convert a vector of routes to the set of required edges it contains
std::set<std::tuple<int, int, int>> routes_to_required_edges(const std::vector<Route>& routes);