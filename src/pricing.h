#pragma once

#include "../pathwyse/core/data/problem.h"
#include "solution.h"
#include <vector>


// Defines the pricing problem for a given vehicle.
// Does not have the updated dual values for the node costs.
// This is used in the initialization of each pricing sub problem
//   @param instance: the instance of the problem
//   @param vehicle: the vehicle that will perform the routes
Problem* create_pricing_instance(const Instance &instance, const Vehicle &vehicle);


// Update a pricing problem with the dual values given by the master problem
void update_pricing_instance(Problem* pricing_problem, const vector<double> &alphas, double beta,
        const Instance &instance, const Vehicle &vehicle);


/* Solves a pre-defined pricing problem and return a vector of routes that are feasible (of maximal reduced cost)
    @param pricing_problem: the pricing problem to solve
    @param pool_size: the maximum number of routes to return
    @param instance: the instance of the problem
    @param vehicle: the vehicle that will perform the routes
*/
vector<Route> solve_pricing_problem(Problem* pricing_problem, int pool_size, const Instance &instance, const Vehicle &vehicle);

