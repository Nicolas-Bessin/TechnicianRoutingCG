#pragma once

#include "instance/instance.h"
#include "master_problem/master.h"
#include "routes/route.h"

#include <vector>


// @brief Computes the delta in the route's cost if we remove a given intervention
// @param route The route from which we want to remove the intervention
// @param intervention The intervention we want to remove
// @param instance The instance of the problem
// The delta is defined as : cost(route) - cost(route without intervention)
// Thus, the bigger the delta, the more advantageous it is to remove the intervention
double compute_delta(const Route& route, int intervention, const Instance& instance);

// @brief Deletes a given intervention from a route
// @param route The route from which we want to remove the intervention
// @param intervention The intervention we want to remove
void delete_intervention(Route& route, int intervention, const Instance& instance);

// Given a vector of routes and a corresponding IntegerSolution
// Repairs the solution by removing duplicate intervention coverings from the route
// If the triangle inequality is respected, this can only improve the solution
// Modifies only the routes used in the initial solution
// Eventually, if after repair a route becomes empty, its coefficient in the solution is set to 0
void repair_routes(std::vector<Route>& routes, IntegerSolution& solution, const Instance& instance);


// Comutes a repaired solution from the initial solution and routes, without modifying the initial solution nor the routes
std::pair<IntegerSolution, std::vector<Route>> compute_repaired_solution(const std::vector<Route>& routes, const IntegerSolution& solution, const Instance& instance);