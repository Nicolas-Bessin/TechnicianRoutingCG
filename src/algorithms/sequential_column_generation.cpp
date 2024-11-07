#include "sequential_column_generation.h"

#include "master_problem/master_solver.h"

#include <iostream>
#include <iomanip>
#include <chrono>

#include "gurobi_c++.h"


CGResult sequential_column_generation(const Instance & instance, std::vector<Route> & routes, const ColumnGenerationParameters & parameters){

    using std::vector, std::string;
    using std::cout, std::endl, std::setprecision;
    namespace chrono = std::chrono;

    auto procedure_start = chrono::steady_clock::now();

    // Create a root node for the algorithm
    BPNode root = RootNode(routes);

    // Create the underlying RMP model - at first with duration only
    vector<GRBVar> route_vars;
    vector<GRBVar> postpone_vars;
    vector<GRBConstr> intervention_ctrs;
    vector<GRBConstr> vehicle_ctrs;
    GRBModel model = create_model(
        instance, 
        routes, 
        route_vars, 
        postpone_vars, 
        intervention_ctrs,
        vehicle_ctrs,
        parameters.solver_objective_mode
    );

    // Main loop of the column generation algorithm
    int iteration = 0;
    // Stopping conditions
    bool stop = false;
    int consecutive_non_improvement = 0;
    double previous_solution_objective = std::numeric_limits<double>::infinity();

    // Master Solutions - Do a first solve before the loop
    int status = solve_model(model);
    MasterSolution solution = extract_solution(model, route_vars, intervention_ctrs, vehicle_ctrs);
    DualSolution& dual_solution = solution.dual_solution;
    DualSolution previous_dual_solution{};


    return CGResult{};
}