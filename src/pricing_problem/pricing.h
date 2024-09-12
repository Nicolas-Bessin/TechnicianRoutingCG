#pragma once


#include "instance/instance.h"
#include "routes/route.h"
#include "master_problem/master.h"

#include <vector>
#include <random>

#define RANDOM_SEED -1

std::vector<Route> solve_pricing_problems_basic(
    const DualSolution & solution,
    const Instance & instance,
    bool using_cyclic_pricing,
    int n_ressources_dominance,
    const std::vector<int> & vehicle_order,
    double reduced_cost_threshold = 1e-6
);


/*
    Generates a new set of route based on the diversitication algorithm presented in Dupin et al. (2021) 
*/
std::vector<Route> solve_pricing_problems_diversification(
    const DualSolution & solution,
    const Instance & instance,
    bool using_cyclic_pricing,
    int n_ressources_dominance,
    const std::vector<int> & vehicle_order,
    double reduced_cost_threshold = 1e-6,
    int seed = RANDOM_SEED
);


std::vector<Route> solve_pricing_problems_clustering(
    const DualSolution & solution,
    const Instance & instance,
    bool using_cyclic_pricing,
    int n_ressources_dominance,
    const std::vector<int> &vehicle_order,
    double reduced_cost_threshold,
    int seed = RANDOM_SEED
    );


std::vector<Route> solve_pricing_problems_tabu_search(
    const DualSolution & solution,
    const Instance & instance,
    bool using_cyclic_pricing,
    int n_ressources_dominance,
    const std::vector<int> &vehicle_order,
    double reduced_cost_threshold,
    int max_iterations,
    int max_modifications,
    int seed = RANDOM_SEED
    );