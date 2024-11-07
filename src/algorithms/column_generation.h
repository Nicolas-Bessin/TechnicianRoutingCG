#pragma once

#include "instance/instance.h"

#include "master_problem/master.h"
#include "master_problem/node.h"

#include "routes/route.h"

#include "algorithms/parameters.h"

#include <vector>

struct CGResult {
    MasterSolution master_solution;
    IntegerSolution integer_solution;
    int number_of_iterations;
    int master_time;
    int pricing_time;
    int integer_time;
    std::vector<double> objective_values;
    std::vector<double> integer_objective_values;
    std::vector<double> solution_costs;
    std::vector<double> integer_solution_costs;
    std::vector<double> covered_interventions;
    std::vector<int> integer_covered_interventions;
    std::vector<int> time_points;
};


std::map<std::string, std::any> pathwyse_parameters_dict(
    const ColumnGenerationParameters& parameters,
    double remaining_time,
    bool using_cyclic_pricing = false
);

/*
    Solve the column generation problem for the given instance and initial node
    using the given initial routes and reduced cost threshold.

    The column generation algorithm will stop when the reduced cost of the best
    column is greater than the reduced_cost_threshold or when the time limit is reached.

    The algorithm will also stop if the number of iterations exceeds max_iterations.

    If compute_integer_solution is set to true, the algorithm will also try to compute
    an integer solution to the problem.

    @param instance: The instance of the problem to solve
    @param initial_routes: The initial routes to use in the column generation algorithm
    @param parameters: The parameters of the column generation algorithm, see parameters.h

    Returns a CGResult object containing the results of the column generation algorithm.
*/
CGResult column_generation(
    const Instance & instance,
    std::vector<Route> & routes,
    const ColumnGenerationParameters & parameters
    );
