#pragma once

#include "../../pathwyse/core/data/problem.h"
#include "../solution.h"
#include <vector>
#include <memory>


// Defines the pricing problem for a given vehicle b wirting it to a file.
// We write the pricing problem under the pathwyse default file format.
//  @param folder : the folder where the pricing problem will be written
//  @param instance: the instance of the problem
//  @param vehicle: the vehicle that will perform the routes
void write_pricing_instance(const std::string &folder, const Instance &instance, const Vehicle &vehicle);



// Loads, updates using the alpha and beta values, and solves a pricing problem.
// @param filepath : the path to the pricing problem file
// @param alphas : the dual values for the node costs
// @param beta : the dual value for the constant term
// @param instance : the instance of the problem
// @param vehicle : the vehicle that will perform the routes
std::vector<Route> solve_pricing_problem_file(const std::string &filepath, const std::vector<double> &alphas, double beta,
    const Instance &instance, const Vehicle &vehicle);