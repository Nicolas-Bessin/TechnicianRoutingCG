#pragma once

#include "../../pathwyse/core/data/problem.h"
#include "instance/instance.h"
#include "routes/route.h"
#include "master_problem/master.h"

#include <vector>
#include <memory>

#include <set>
#include <tuple>


// Solve the pricing problem for a given vehicle using the basic Pathwyse algorithm
Route solve_pricing_problem(
    const Instance &instance, 
    const Vehicle &vehicle,
    const DualSolution &dual_solution,
    bool use_maximisation_formulation,
    bool use_cyclic_pricing = false,
    int n_res_dom = -1
    );


std::vector<Route> solve_pricing_problem_pulse(
    const Instance &instance, 
    const Vehicle &vehicle,
    const DualSolution &dual_solution,
    bool use_maximisation_formulation,
    int delta = 10,
    int pool_size = 10,
    bool verbose = false
    );


std::vector<Route> solve_pricing_problem_pulse_grouped(
    const Instance &instance, 
    const std::vector<int> & vehicle_indexes,
    const DualSolution &dual_solution,
    bool use_maximisation_formulation,
    int delta,
    int pool_size,
    bool verbose = false
    );

std::vector<Route> solve_pricing_problem_pulse_parallel(
    const Instance &instance, 
    const Vehicle &vehicle,
    const DualSolution &dual_solution,
    bool use_maximisation_formulation,
    int delta = 10,
    int pool_size = 10,
    bool verbose = false
    );


std::vector<Route> solve_pricing_problem_pulse_grouped_par(
    const Instance &instance, 
    const std::vector<int> & vehicle_indexes,
    const DualSolution &dual_solution,
    bool use_maximisation_formulation,
    int delta,
    int pool_size,
    bool verbose = false
    );