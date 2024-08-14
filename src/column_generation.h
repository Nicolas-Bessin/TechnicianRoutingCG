#pragma once

#include "instance.h"
#include "master.h"
#include "branch_and_price.h"

#include <vector>

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
        const BPNode& node,
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

    @param instance The instance of the problem to solve.
    @param node The current node in the B&P tree (is modified when adding routes)
    @param routes The routes before this round of column generation (is modified when adding routes)
    @param reduced_cost_threshold Only accepts columns with reduced cost over this threshold
    @param time_limit The time limit for the column generation algorithm (in seconds)
    @param max_iterations The maximum number of iterations for the column generation algorithm
    @param compute_integer_solution If set to true, compute an Integer solution at the end
    @param verbose If set to true, prints information at each iteration

    Returns a CGResult object containing the results of the column generation algorithm.
*/
CGResult column_generation(
    const Instance & instance,
    BPNode & node,
    std::vector<Route> & initial_routes,
    double reduced_cost_threshold,
    int time_limit = 60, // (in seconds)
    int max_iterations = 1000,
    bool switch_to_cyclic_pricing = false,
    bool compute_integer_solution = true,
    bool verbose = false
    );
