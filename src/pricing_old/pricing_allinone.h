#pragma once

#include "../solution.h"
#include <vector>

std::vector<Route> create_solve_pricing_instance(const std::vector<double> &alphas, double beta, const Instance &instance, const Vehicle &vehicle, int pool_size);