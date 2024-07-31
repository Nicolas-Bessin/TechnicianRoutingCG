#include "src/parser.h"
#include "src/preprocessing.h"
#include "src/column_generation.h"
#include "src/analysis.h"
#include "src/compact_solver.h"

#include <memory>   
#include <iostream>
#include <iomanip>
#include <chrono>

#include <algorithm>
#include <random>

#define SCALE_FACTOR 1
#define TIME_LIMIT 90
#define SOLVER_MODE WARM_START
#define THRESHOLD 1e-6
#define VERBOSE true

int main(int argc, char *argv[]){

    using std::cout, std::endl;
    using std::setprecision, std::fixed;
    using std::vector, std::string, std::to_string, std::pair;
    using std::unique_ptr;
    namespace chrono = std::chrono;

    // Parse the instance from a JSON file
    auto start_parse = chrono::steady_clock::now();
    cout << "Technician Routing Problem using Column Generation" << endl;
    string default_filename = "../data/instance_1_all_feasible.json";
    Instance instance = parse_file(default_filename);

    preprocess_interventions(instance);
    auto end_parse = chrono::steady_clock::now();
    int diff_parse = chrono::duration_cast<chrono::milliseconds>(end_parse - start_parse).count();

    
    cout << "Total time spent parsing the instance : " << diff_parse << " ms" << endl;

    // Create a dummy route for the first vehicle
    vector<Route> routes;
    routes.push_back(Route(0, instance.number_interventions));

    // Do a first round of column generation
    CGResult initial_cg = column_generation(instance, routes, THRESHOLD, TIME_LIMIT, VERBOSE);

    // Print the time it took to solve the master problem
    cout << "Total time spent solving the master problem : " << initial_cg.master_time << " ms" << endl;
    cout << "Total time spent solving the pricing problems : " << initial_cg.pricing_time << " ms" << endl;
    cout << "Total time spent solving the integer problem : " << initial_cg.integer_time << " ms" << endl;
    cout << "-----------------------------------" << endl;

    // Print the routes in the integer solution (in detail)
    //print_used_routes(integer_solution, routes, instance);

    int remaining_time = TIME_LIMIT - (initial_cg.master_time + initial_cg.pricing_time + initial_cg.integer_time) / 1000;
    // Keep only the routes that are used in the integer solution
    vector<Route> used_routes = keep_used_routes(initial_cg.routes, initial_cg.integer_solution);
    // We can then solve the compact model with these imposed routings
    CompactSolution compact_solution = compact_solver(instance, remaining_time, used_routes, SOLVER_MODE);
    // Convert back to routes
    vector<Route> compact_routes = compact_solution_to_routes(instance, compact_solution);
    // Create a dummy integer solution (all variables set to 1)
    IntegerSolution compact_integer_solution = IntegerSolution(vector<int>(compact_routes.size(), 1), compact_solution.objective_value);

    cout << "Manual computing of the compact solution value : " << compute_integer_objective(compact_integer_solution, compact_routes, instance) << endl;

    cout << "-----------------------------------" << endl;
    // Print the routes in the compact solution
    //print_used_routes(compact_integer_solution, compact_routes, instance);

    // Run the analysis on the compact solution
    full_analysis(compact_integer_solution, compact_routes, instance);

    return 0;
}


/*
    // Now, we try to create a route for an unused vehicle that cover interventions not already covered
    const int vehicle_id = 2;
    cout << "Creating a new route for vehicle " << vehicle_id << endl;
    Vehicle newV2 = vehicle_mask(instance.vehicles[vehicle_id], covered_interventions(integer_solution, routes, instance));
    // We know generate & solve the pricing problem for this new vehicle
    unique_ptr<Problem> new_pricing_problem = create_pricing_instance(instance, newV2, SCALE_FACTOR);
    // Update with alpahs and beta at 0 for now
    update_pricing_instance(new_pricing_problem, vector<double>(instance.number_interventions, 0), 0, instance, newV2, SCALE_FACTOR);
    // Solve the pricing problem
    vector<Route> new_routes_v2 = solve_pricing_problem(new_pricing_problem, 5, instance, newV2);
    // Print the cost of the new routes
    for (const auto &route : new_routes_v2){
        print_route(route, instance);
    }
    // Add the new route to the master problem
    for (const auto &route : new_routes_v2){
        routes.push_back(route);
    }
    // Re-solve the integer problem
    MasterSolution master_solution_v2 = cg_solver(instance, routes, 60);
    cout << "New master solution found with objective value : " << master_solution_v2.objective_value << endl;
    IntegerSolution integer_solution_v2 = solve_integer_problem(instance, routes);
    cout << "New integer solution found with objective value : " << integer_solution_v2.objective_value << endl;
    cout << "-----------------------------------" << endl;
    print_non_covered_interventions(integer_solution_v2, routes, instance, true);
*/



