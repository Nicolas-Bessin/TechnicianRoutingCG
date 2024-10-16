#pragma once

#include "instance/instance.h"
#include "master_problem/master.h"
#include "routes/route.h"
#include "algorithms/parameters.h"
#include "algorithms/column_generation.h"

#include <string>

void export_solution(
    const std::string& filename, 
    const Instance& instance, 
    const CGResult& result,
    const std::vector<Route>& routes,
    const ColumnGenerationParameters& parameters
);