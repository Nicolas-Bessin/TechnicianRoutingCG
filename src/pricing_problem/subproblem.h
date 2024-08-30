#pragma once

#include "../../pathwyse/core/data/problem.h"
#include "instance/instance.h"
#include "routes/route.h"
#include "master_problem/master.h"

#include <vector>
#include <memory>

#include <set>
#include <tuple>


// Defines the pricing subproblem for a given vehicle.
// Does not have the updated dual values for the node costs.
// This is used in the initialization of each pricing sub problem
//   @param instance: the instance of the problem
//   @param vehicle: the vehicle that will perform the routes
std::unique_ptr<Problem> create_pricing_instance(
    const Instance &instance, 
    const Vehicle &vehicle,
    bool use_cyclic_pricing = false,
    const std::set<std::tuple<int, int, int>> &forbidden_edges = {},
    const std::set<std::tuple<int, int, int>> &required_edges = {}
    );


// Update a pricing subproblem with the dual values given by the master problem
void update_pricing_instance(
    std::unique_ptr<Problem> & pricing_problem, 
    const MasterSolution& master_solution, 
    const Instance &instance, 
    const Vehicle &vehicle
    );


/* Solves a pre-defined pricing subproblem and return a route that are feasible (of maximal reduced cost)
    @param pricing_problem: the pricing problem to solve
    @param n_res_dom : the number of resources to use in the dominance test (default -1 : use all resources)
    @param instance: the instance of the problem
    @param vehicle: the vehicle that will perform the routes
*/
Route solve_pricing_problem(
    std::unique_ptr<Problem> & pricing_problem,
    const Instance &instance, 
    const Vehicle &vehicle,
    int n_res_dom = -1
    );


