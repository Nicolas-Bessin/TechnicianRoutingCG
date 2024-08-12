#pragma once

#include "instance.h"
#include "route.h"
#include "pricing.h"
#include "branch_and_price.h"

#include <vector>

/*
    Greedy heuristic to generate a set of routes that cover as much interventions as possible at a given BPNode
    Begin with the first vehicle in the given order, generate a feasible route that maximizes the benefit.
    Then, continue with each successive vheicle until either all interventions are covered or no more vehicles are available.
    If no vehicle order is given, the vehicles are explored from the one who can do the least interventions to the one who can do the most.
*/
std::vector<Route> greedy_heuristic(
    const Instance& instance, 
    std::vector<int> vehicle_order = std::vector<int>{},
    const std::set<std::tuple<int, int, int>>& forbidden_edges = {},
    const std::set<std::tuple<int, int, int>>& required_edges = {}
    );