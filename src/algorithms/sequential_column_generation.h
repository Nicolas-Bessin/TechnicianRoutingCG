#pragma once

#include "instance/instance.h"

#include "master_problem/master.h"
#include "master_problem/node.h"

#include "routes/route.h"

#include "algorithms/parameters.h"

#include "algorithms/column_generation.h"

#include <vector>


CGResult sequential_column_generation(const Instance & instance, std::vector<Route> & routes, const ColumnGenerationParameters & parameters);