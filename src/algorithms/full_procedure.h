#pragma once

#include "parameters.h"
#include "column_generation.h"


CGResult full_cg_procedure(const Instance & instance, std::vector<Route> & routes, const ColumnGenerationParameters & parameters);