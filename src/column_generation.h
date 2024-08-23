#pragma once

#include "instance.h"
#include "master.h"
#include "branch_and_price.h"

#include <vector>

#define ALL_RES_DOMINANCE -1

struct CGResult {
    MasterSolution master_solution;
    IntegerSolution integer_solution;
    int number_of_iterations;
    int master_time;
    int pricing_time;
    int integer_time;
    int building_time;
    // Empty constructor
    CGResult() {};
    // Constructor
    CGResult(
        const MasterSolution& master_solution,
        const IntegerSolution& integer_solution,
        const std::vector<Route>& routes,
        int number_of_iterations,
        int master_time,
        int pricing_time,
        int integer_time,
        int building_time
    ) : 
        master_solution(master_solution),
        integer_solution(integer_solution),
        number_of_iterations(number_of_iterations),
        master_time(master_time),
        pricing_time(pricing_time),
        integer_time(integer_time),
        building_time(building_time)
    {}
};

/*
    Solve the column generation problem for the given instance and initial node
    using the given initial routes and reduced cost threshold.

    The column generation algorithm will stop when the reduced cost of the best
    column is greater than the reduced_cost_threshold or when the time limit is reached.

    The algorithm will also stop if the number of iterations exceeds max_iterations.

    If compute_integer_solution is set to true, the algorithm will also try to compute
    an integer solution to the problem.

    @param instance: The instance of the problem to solve
    @param node: The root node of the branch and price tree
    @param initial_routes: The initial routes to use in the column generation algorithm

    @param max_resources_dominance: The maximum number of resources to use in the dominance test
    @param switch_to_cyclic_pricing: Whether to switch to cyclic pricing when no new routes are added
    @param compute_integer_solution: Whether to compute an integer solution to the problem
    @param time_limit: The time limit for the column generation algorithm
    @param reduced_cost_threshold: The reduced cost threshold to stop the column generation algorithm
    @param verbose: Whether to print information about the column generation algorithm

    Returns a CGResult object containing the results of the column generation algorithm.
*/
CGResult column_generation(
    const Instance & instance,
    BPNode & node,
    std::vector<Route> & routes,
    int max_resources_dominance = ALL_RES_DOMINANCE,
    bool switch_to_cyclic_pricing = false,
    bool compute_integer_solution = true,
    int time_limit = 60, // (in seconds)
    double reduced_cost_threshold = 1e-6,
    bool verbose = false
    );
