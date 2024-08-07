#pragma once

#include "instance.h"
#include "solution.h"
#include "pricing.h"
#include <vector>

struct CGResult {
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
    std::vector<Route> initial_routes, 
    double reduced_cost_threshold, 
    int time_limit = 60, 
    bool verbose = false,
    bool compute_integer_solution = true
    );