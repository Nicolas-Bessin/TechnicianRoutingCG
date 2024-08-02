#pragma once

#include "instance.h"
#include "solution.h"
#include "pricing.h"

#include <vector>

/*
    Greedy heuristic to generate a set of routes that cover as much interventions as possible
    Begin with the first vehicle, generate a feasible route that maximizes the benefit.
    Then, continue with each successive vheicle until either all interventions are covered or no more vehicles are available.
*/
std::vector<Route> greedy_heuristic(const Instance& instance);