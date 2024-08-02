#include "src/parser.h"
#include "src/preprocessing.h"

#include "src/column_generation.h"

#include "src/analysis.h"

#include "src/compact_solution.h"
#include "src/compact_solver.h"
#include "src/solution_converter.h"

#include <memory>   
#include <iostream>
#include <iomanip>
#include <chrono>

#define TIME_LIMIT 60
#define SOLVER_MODE WARM_START
#define THRESHOLD 1e-6
#define VERBOSE false

int main(int argc, char *argv[]){

    using std::cout, std::endl;
    using std::setprecision, std::fixed;
    using std::vector, std::string, std::to_string, std::pair;
    using std::unique_ptr;
    namespace chrono = std::chrono;

    // Parse the instance from a JSON file
    auto start_parse = chrono::steady_clock::now();
    cout << "Technician Routing Problem using Column Generation" << endl;
    cout << "-----------------------------------" << endl;
    string default_filename = "../data/instance_1_all_feasible.json";
    Instance instance = parse_file(default_filename);


    // Check wether the time and distance matrices are symetric
    cout << "Distance matrix is symetric : " << is_symmetric(instance.distance_matrix) << " - Biggest gap : " << symmetry_gap(instance.distance_matrix) << endl;
    cout << "Time matrix is symetric : " << is_symmetric(instance.time_matrix) << " - Biggest gap : " << symmetry_gap(instance.time_matrix) << endl;

    preprocess_interventions(instance);

    auto end_parse = chrono::steady_clock::now();
    int diff_parse = chrono::duration_cast<chrono::milliseconds>(end_parse - start_parse).count();

    cout << "Total time spent parsing the instance : " << diff_parse << " ms" << endl;
    cout << "-----------------------------------" << endl;
    cout << "Starting the column generation algorithm" << endl;

    // Create a dummy route for the first vehicle
    vector<Route> initial_routes;
    initial_routes.push_back(Route(0, instance.number_interventions));

    // Global time limit for the column generation algorithm of 60 seconds
    const int time_limit = TIME_LIMIT * 1000;

    CGResult result = column_generation(instance, initial_routes, THRESHOLD, time_limit, VERBOSE);

    // Extract the results from the column generation algorithm
    int master_time = result.master_time;
    int pricing_time = result.pricing_time;
    int integer_time = result.integer_time;
    int sub_building_time = result.building_time;
    vector<Route> routes = result.routes;
    MasterSolution master_solution = result.master_solution;
    IntegerSolution integer_solution = result.integer_solution;
    

    // If the integer solution is not feasible, we can't do much
    if (!integer_solution.is_feasible){
        cout << "-----------------------------------" << endl;
        cout << "The integer solution is not feasible" << endl;
        return 0;
    }
    
    // Print the routes in the integer solution (in detail)
    full_analysis(integer_solution, routes, instance);

    int elapsed_time = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start_parse).count();
    // Print the time it took to solve the master problem
    cout << "-----------------------------------" << endl;
    cout << "Total time spent building the pricing problems : " << sub_building_time << " ms" << endl;
    cout << "Total time spent solving the master problem : " << master_time << " ms" << endl;
    cout << "Total time spent solving the pricing problems : " << pricing_time << " ms - Average : " << pricing_time / result.number_of_iterations << " ms" << endl;
    cout << "Total time spent solving the integer problem : " << integer_time << " ms" << endl;
    cout << "Total elapsed time : " << elapsed_time << " ms" << endl;
    cout << "-----------------------------------" << endl;

    // cout << " Starting the compact solver using mode " << SOLVER_MODE << endl;

    
    // int remaining_time = TIME_LIMIT - elapsed_time / 1000;
    // // Keep only the routes that are used in the integer solution
    // vector<Route> used_routes = keep_used_routes(routes, integer_solution);
    // // We can then solve the compact model with these imposed routings
    // CompactSolution<int> compact_solution = compact_solver(instance, remaining_time, used_routes, SOLVER_MODE, VERBOSE);
    // // Convert back to routes
    // vector<Route> compact_routes = compact_solution_to_routes(instance, compact_solution);
    // // Create a dummy integer solution (all variables set to 1)
    // IntegerSolution compact_integer_solution = IntegerSolution(vector<int>(compact_routes.size(), 1), compact_solution.objective_value);

    // cout << "Manual computing of the compact solution value : " << compute_integer_objective(compact_integer_solution, compact_routes, instance) << endl;

    // cout << "-----------------------------------" << endl;
    // // Print the routes in the compact solution
    // //print_used_routes(compact_integer_solution, compact_routes, instance);

    // // Run the analysis on the compact solution
    // full_analysis(compact_integer_solution, compact_routes, instance);

    // Convert our master solution to a compact solution
    CompactSolution<double> compact_solution = to_compact_solution(master_solution, routes, instance);
    // Evaluate the objective value of the compact solution
    double compact_objective = evaluate_compact_solution(compact_solution, instance);
    cout << "Objective value of the relaxed converted compact solution : " << compact_objective << endl;

    // Do the same thing for the integer solution
    CompactSolution<int> compact_integer_solution = to_compact_solution(integer_solution, routes, instance);
    // Evaluate the objective value of the compact solution
    double compact_integer_objective = evaluate_compact_solution(compact_integer_solution, instance);
    cout << "Objective value of the integer converted compact solution : " << compact_integer_objective << endl;


    // // We now want to get a solution to the relaxed compact formulation
    // CompactSolution<double> relaxed_compact_solution = relaxed_compact_solver(instance, TIME_LIMIT, true);
    // // Evaluate the objective value of the relaxed compact solution
    // double relaxed_compact_objective = evaluate_compact_solution(relaxed_compact_solution, instance);
    // cout << "Objective value of the relaxed compact solution : " << relaxed_compact_objective << endl;

    return 0;
}


    


