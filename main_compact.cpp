#include "src/instance.h"
#include "src/parser.h"
#include "src/preprocessing.h"

#include "src/heuristics.h"
#include "src/compact_solver.h"

#include "src/solution_converter.h"

#include "src/analysis.h"

#include <iostream>
#include <string>
#include <chrono>
#include <iomanip>
#include <memory>

#define TIME_LIMIT 300
#define HEURISTIC_INIT false
#define N_INTERVENTIONS 30

int main(int argc, char** argv) {
    using std::cout, std::endl;
    using std::setprecision, std::fixed;
    using std::vector, std::string, std::to_string;
    using std::unique_ptr;
    namespace chrono = std::chrono;

    // Parse the instance from a JSON file
    auto start_parse = chrono::steady_clock::now();
    cout << "Technician Routing Problem using the compact formulation" << endl;
    string default_filename = "../data/instance_1.json";
    Instance instance = parse_file(default_filename, false);

    // ONly keep the first N_INTERVENTIONS nodes
    vector<int> kept_nodes = vector<int>(instance.number_interventions);
    for (int i = 0; i < N_INTERVENTIONS; i++){
        kept_nodes[i] = 1;
    }
    instance = cut_instance(instance, kept_nodes);

    preprocess_interventions(instance);
    auto end_parse = chrono::steady_clock::now();
    int diff_parse = chrono::duration_cast<chrono::milliseconds>(end_parse - start_parse).count();

    cout << "-----------------------------------" << endl;
    // Define a vector of routes to help the solver
    vector<Route> initial_routes;
    if (HEURISTIC_INIT){
        cout << "Initializing the routes with a greedy heuristic" << endl;
        initial_routes = greedy_heuristic(instance);
        // Print the objective value of the greedy solution
        IntegerSolution greedy_solution = IntegerSolution(vector<int>(initial_routes.size(), 1), 0);
        double greedy_objective = compute_integer_objective(greedy_solution, initial_routes, instance);
        cout << "Objective value of the greedy solution : " << setprecision(2) << fixed << greedy_objective << endl;
    } else {
        cout << "No heuristic initialization" << endl;
    }
    cout << "-----------------------------------" << endl;


    // Solve the problem using the compact formulation
    auto start_solve = chrono::steady_clock::now();
    CompactSolution compact_solution = compact_solver(instance, TIME_LIMIT, initial_routes, WARM_START, true);
    auto end_solve = chrono::steady_clock::now();

    cout << "-----------------------------------" << endl;

    // Convert the compact solution to a set of routes
    vector<Route> routes = compact_solution_to_routes(instance, compact_solution);

    // Check the feasibility of the solution
    // Build a dummy Integer solution : all the coefficients are 1 (we only returned the routes that were used)
    vector<int> coefficients(routes.size(), 1);
    IntegerSolution integer_solution(coefficients, 0.0);

    // Analyze the solution
    // Solution analysis
    // Recompute the objective value
    double objective_value = compute_integer_objective(integer_solution, routes, instance);
    cout << "Objective value : " << setprecision(2) << fixed << objective_value << endl;
    cout << "Number of covered interventions : " << count_covered_interventions(integer_solution, routes, instance);
    cout << " / " << instance.number_interventions << endl;

    cout << "Number of used vehicles : " << count_used_vehicles(integer_solution, routes, instance);
    cout << " / " << instance.vehicles.size() << endl;

    cout << "Number of interventions that could be covered : " << count_coverable_interventions(integer_solution, routes, instance) << endl;

    // Check that all used routes are feasible
    bool all_feasible = true;
    for (int i = 0; i < routes.size(); i++){
        const Route& route = routes.at(i);
        if (integer_solution.coefficients[i] > 0 && !is_route_feasible(route, instance)){
            cout << "Route " << i << " is not feasible" << endl;
            all_feasible = false;
        }
    }
    if (all_feasible){
        cout << "All routes are feasible" << endl;
    }

    cout << "Number of routes with duplicates : " << count_routes_with_duplicates(routes) << " / " << routes.size() << endl;
    cout << "Number of used routes with duplicates : " << count_used_routes_with_duplicates(integer_solution, routes) << endl;

    cout << "Number of route kilometres : " << count_kilometres_travelled(integer_solution, routes, instance) << " km" << endl;

    cout << "Time spent travelling : " << time_spent_travelling(integer_solution, routes, instance) << " minutes" << endl;
    cout << "Time spent working : " << time_spent_working(integer_solution, routes, instance) << " minutes" << endl;
    cout << "Time spent waiting : " << time_spent_waiting(integer_solution, routes, instance) << " minutes" << endl;

    cout << "-----------------------------------" << endl;
    //print_used_routes(integer_solution, routes, instance); 
    print_non_covered_interventions(integer_solution, routes, instance, false);
    cout << "-----------------------------------" << endl;
    print_used_vehicles(integer_solution, routes, instance);
    print_vehicles_non_covered(integer_solution, routes, instance);
    cout << "-----------------------------------" << endl;


    return 0;
}