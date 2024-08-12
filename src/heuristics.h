#pragma once

#include "instance.h"
#include "route.h"
#include "pricing.h"
#include "branch_and_price.h"

#include <vector>

/*
    Greedy heuristic to generate a set of routes that cover as much interventions as possible
    Begin with the first vehicle, generate a feasible route that maximizes the benefit.
    Then, continue with each successive vheicle until either all interventions are covered or no more vehicles are available.
*/
std::vector<Route> greedy_heuristic(const Instance& instance);

/* 
    Greedy heuristic to generate a set of routes that cover as much interventions as possible
    Begin with the first vehicle, generate a feasible route that maximizes the benefit.
    Then, continue with each successive vheicle until either all interventions are covered or no more vehicles are available.
    This version of the greedy heuristic uses the alphas to penalize interventions already covered.
*/
std::vector<Route> greedy_heuristic_alphas(const Instance& instance);

/*
    Greedy heuristic to generate a set of routes that cover as much interventions as possible at a given BPNode
    Begin with the first vehicle, generate a feasible route that maximizes the benefit.
    Then, continue with each successive vheicle until either all interventions are covered or no more vehicles are available.
*/
std::vector<Route> greedy_heuristic_bpnode(const Instance& instance, const BPNode& node);