#pragma once


#include "instance/instance.h"
#include "routes/route.h"
#include "master_problem/master.h"

#include <vector>
#include <random>

std::vector<Route> solve_pricing_problems_basic(
    const DualSolution & solution,
    const Instance & instance,
    bool using_cyclic_pricing,
    int n_ressources_dominance,
    double reduced_cost_threshold = 1e-6
);


// Generates a random cycle permutation of [|0, n-1|]
// @param n: the number of elements in the cycle
// @param seed: the seed for the random number generator
std::vector<int> random_cycle(int n, int seed = 0);

/*
    Generates a new set of route based on the diversitication algorithm presented in Dupin et al. (2021) 
*/
std::vector<Route> solve_pricing_problems_diversification(
    const DualSolution & solution,
    const Instance & instance,
    bool using_cyclic_pricing,
    int n_ressources_dominance,
    double reduced_cost_threshold = 1e-6,
    int seed = 0
);