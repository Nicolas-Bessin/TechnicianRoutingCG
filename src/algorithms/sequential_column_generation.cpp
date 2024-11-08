#include "sequential_column_generation.h"

#include "master_problem/master_solver.h"

#include <iostream>
#include <iomanip>
#include <chrono>

#include "gurobi_c++.h"


SequentialCGResult sequential_column_generation(const Instance & instance, std::vector<Route> & routes, const ColumnGenerationParameters & initial_parameters){

    using std::vector, std::string;
    using std::cout, std::endl, std::setprecision;
    namespace chrono = std::chrono;

    auto procedure_start = chrono::steady_clock::now();

    ColumnGenerationParameters parameters = initial_parameters;

    // Column Generation in the duration only mode
    cout << "-----------------------------------" << endl;
    cout << "Starting phase 1 : minimising outsourced duration" << endl;
    parameters.solver_objective_mode = SolverMode::DURATION_ONLY;
    parameters.compute_integer_solution = false;
    CGResult result_phase_1 = column_generation(instance, routes, parameters);
    // The value we want is the last relaxed objective value
    double max_outsourced_duration = result_phase_1.objective_values.back();

    // Column Generation in the solution minimisation mode
    cout << "-----------------------------------" << endl;
    cout << "Starting phase 2 : minimising solution cost" << endl;
    parameters.solver_objective_mode = SolverMode::SOLUTION_MINIMISATION;
    parameters.compute_integer_solution = true;
    parameters.max_outsourced_duration = max_outsourced_duration;
    CGResult result_phase_2 = column_generation(instance, routes, parameters);

    // Combine the results
    CGResult result = result_phase_1;
    for (int i = 0; i < result_phase_2.objective_values.size(); i++){
        result.time_points.push_back(result_phase_1.time_points.back() + result_phase_2.time_points[i]);
        result.objective_values.push_back(result_phase_2.objective_values[i]);
        result.solution_costs.push_back(result_phase_2.solution_costs[i]);
        result.covered_interventions.push_back(result_phase_2.covered_interventions[i]);
    }
    // Add the eventual intermediary integer solutions
    result.integer_objective_values.insert(result.integer_objective_values.end(), result_phase_2.integer_objective_values.begin(), result_phase_2.integer_objective_values.end());
    result.integer_solution_costs.insert(result.integer_solution_costs.end(), result_phase_2.integer_solution_costs.begin(), result_phase_2.integer_solution_costs.end());
    result.integer_covered_interventions.insert(result.integer_covered_interventions.end(), result_phase_2.integer_covered_interventions.begin(), result_phase_2.integer_covered_interventions.end());
    // Set the solution to those of the phase 2
    result.master_solution = result_phase_2.master_solution;
    result.integer_solution = result_phase_2.integer_solution;
    result.master_time = result_phase_1.master_time + result_phase_2.master_time;
    result.pricing_time = result_phase_1.pricing_time + result_phase_2.pricing_time;
    result.number_of_iterations = result_phase_1.number_of_iterations + result_phase_2.number_of_iterations;

    return SequentialCGResult{
        result,
        result_phase_1.time_points.back(),
        result_phase_1.number_of_iterations
    };
}