#pragma once

#include "instance/instance.h"
#include "master_problem/master.h"
#include "routes/route.h"
#include "algorithms/parameters.h"

#include <string>

void export_solution(
    const std::string& filename, 
    const Instance& instance, 
    const IntegerSolution& solution, 
    const std::vector<Route>& routes, 
    int elapsed_time, 
    const ColumnGenerationParameters& parameters,
    const std::vector<double>& objective_values,
    const std::vector<int>& objective_time_points
);