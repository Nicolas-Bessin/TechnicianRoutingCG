#pragma once

#include "instance/instance.h"

#include "master_problem/master.h"
#include "master_problem/node.h"

#include "routes/route.h"

#include "algorithms/parameters.h"

#include "algorithms/column_generation.h"

#include <vector>


struct SequentialCGResult {
    CGResult combined_result;
    double phase_1_time;
    int phase_1_iterations;
};


SequentialCGResult sequential_column_generation(const Instance & instance, std::vector<Route> & routes, const ColumnGenerationParameters & parameters);