#pragma once

#include "solution.h"


// Function to find the best route for a given vehicle, given the dual values associated with the interventions
Route find_best_route(const Instance &instance, const Vehicle &vehicle, const vector<double> &alphas, const double beta);