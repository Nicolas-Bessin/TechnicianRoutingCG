#pragma once

#include "instance.h"
#include "solution.h"
#include "pricing.h"
#include "branch_and_price.h"

#include <vector>

struct CGResult {
    BPNode node;
    MasterSolution master_solution;
    IntegerSolution integer_solution;
    std::vector<Route> routes;
    int number_of_iterations;
    int master_time;
    int pricing_time;
    int integer_time;
    int building_time;
};


CGResult column_generation(
    const Instance & instance,
    const BPNode & initial_node,
    const std::vector<Route> initial_routes,
    double reduced_cost_threshold,
    int time_limit = 60,
    int max_iterations = 1000,
    bool compute_integer_solution = true,
    bool verbose = false
    );
