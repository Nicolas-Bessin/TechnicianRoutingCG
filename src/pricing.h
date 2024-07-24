#pragma once

#include "../pathwyse/core/data/problem.h"
#include "solution.h"
#include <vector>
#include <memory>


// Defines the pricing problem for a given vehicle.
// Does not have the updated dual values for the node costs.
// This is used in the initialization of each pricing sub problem
//   @param instance: the instance of the problem
//   @param vehicle: the vehicle that will perform the routes
std::unique_ptr<Problem> create_pricing_instance(const Instance &instance, const Vehicle &vehicle, int scale_factor = 1);


// Update a pricing problem with the dual values given by the master problem
void update_pricing_instance(std::unique_ptr<Problem> & pricing_problem, const std::vector<double> &alphas, double beta,
        const Instance &instance, const Vehicle &vehicle, int scale_factor = 1);


/* Solves a pre-defined pricing problem and return a std::vector of routes that are feasible (of maximal reduced cost)
    @param pricing_problem: the pricing problem to solve
    @param pool_size: the maximum number of routes to return
    @param instance: the instance of the problem
    @param vehicle: the vehicle that will perform the routes
*/
std::vector<Route> solve_pricing_problem(std::unique_ptr<Problem> & pricing_problem, int pool_size, const Instance &instance, const Vehicle &vehicle);


