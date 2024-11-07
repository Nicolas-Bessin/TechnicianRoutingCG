#include "sequential_column_generation.h"

#include "master_problem/master_solver.h"

#include <iostream>
#include <iomanip>
#include <chrono>

#include "gurobi_c++.h"


CGResult sequential_column_generation(const Instance & instance, std::vector<Route> & routes, const ColumnGenerationParameters & initial_parameters){

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

    return result_phase_2;
}