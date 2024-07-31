#include "src/parser.h"
#include "src/preprocessing.h"
#include "src/master.h"
#include "src/pricing.h"
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
    cout << "Distance matrix is symetric : " << is_symetric(instance.distance_matrix) << endl;
    cout << "Time matrix is symetric : " << is_symetric(instance.time_matrix) << endl;

    preprocess_interventions(instance);

    auto end_parse = chrono::steady_clock::now();
    int diff_parse = chrono::duration_cast<chrono::milliseconds>(end_parse - start_parse).count();

    cout << "Total time spent parsing the instance : " << diff_parse << " ms" << endl;
    // Create a dummy route for the first vehicle
    vector<Route> routes;
    routes.push_back(Route(0, instance.number_interventions));

        auto start_sub_building = chrono::steady_clock::now();
    // Create the pricing sub problems for each vehicle
    const string pricing_folder = "../pricing_instances/";
    vector<unique_ptr<Problem>> pricing_problems;
    for (const Vehicle& vehicle : instance.vehicles){
        string filename = pricing_folder + "v_" + to_string(vehicle.id) + ".txt";
        //write_pricing_instance(filename, instance, vehicle);
        pricing_problems.push_back(create_pricing_instance(instance, vehicle));
    }
    auto end_sub_building = chrono::steady_clock::now();
    int diff_sub_building = chrono::duration_cast<chrono::milliseconds>(end_sub_building - start_sub_building).count();
    cout << "Total time spent building the pricing sub problems : " << diff_sub_building << " ms" << endl;

    cout << "-----------------------------------" << endl;
    cout << "Starting the column generation algorithm" << endl;
    int master_time = 0;
    int pricing_time = 0;

    // Global time limit for the column generation algorithm of 60 seconds
    const int time_limit = TIME_LIMIT * 1000;

    // Count the number of time each vehicle's sub problem reached the time limit
    vector<int> time_limit_reached(instance.vehicles.size(), 0);

    // Main loop of the column generation algorithm
    int iteration = 0;
    bool stop = false;
    MasterSolution solution;
    // Testing out the sequential pricing problem solving
    int current_vehicle_index = 0;

    while (!stop && master_time + pricing_time < time_limit){
        // Solve the master problem
        auto start = chrono::steady_clock::now();
        solution = relaxed_RMP(instance, routes);
        auto end = chrono::steady_clock::now();
        int diff = chrono::duration_cast<chrono::milliseconds>(end - start).count();
        if (VERBOSE) {
            cout << "Master problem solved in " << diff << " ms \n";
            cout << "Number of interventions covered : " << setprecision(2) << count_covered_interventions(solution, routes, instance);
            cout << " - Number of zeros in alphas : " << count_zeros(solution.alphas);
            cout << " - Number of zeros in betas : " << count_zeros(solution.betas) << "\n";
        }
        master_time += diff;

        // Solve each pricing sub problem
        auto start_pricing = chrono::steady_clock::now();
        int n_added_routes = 0;

        double max_reduced_cost = 0;


        for (int v = 0; v < instance.vehicles.size(); v++){
            const Vehicle& vehicle = instance.vehicles.at(v);
            update_pricing_instance(pricing_problems.at(v), solution.alphas, solution.betas[v], instance, vehicle);
            vector<Route> best_new_routes = solve_pricing_problem(pricing_problems.at(v), 5, instance, vehicle);
            if (best_new_routes.size() == 0){
                time_limit_reached[vehicle.id]++;
            }
            // Go through the returned routes, and add them to the master problem if they have a positive reduced cost
            for (const auto &route : best_new_routes){
                max_reduced_cost = std::max(max_reduced_cost, route.reduced_cost);
                //double computed_reduced_cost = compute_reduced_cost(route, solution.alphas, solution.betas[v], instance);
                if (route.reduced_cost > THRESHOLD){
                    routes.push_back(route);
                    n_added_routes++;
                }
            }
        }

        auto end_pricing = chrono::steady_clock::now();
        int diff_pricing = chrono::duration_cast<chrono::milliseconds>(end_pricing - start_pricing).count();
        pricing_time += diff_pricing;
        if (VERBOSE) {
            cout << "Pricing sub problems solved in " << diff_pricing << " ms - Added " << n_added_routes << " routes";
            cout << " - Max reduced cost : " << setprecision(15) << max_reduced_cost << "\n";
            pricing_time += diff_pricing;
            cout << "Iteration " << iteration << " - Objective value : " << solution.objective_value << "\n";
        }
        // If no route was added, we stop the algorithm
        if (n_added_routes == 0){
            stop = true;
        }
        iteration++;
    }

    cout << "-----------------------------------" << endl;
    if (stop) {
        cout << "Found no new route to add" << endl;
    }
    cout << "End of the column generation after " << iteration << " iterations" << endl;
    cout << "Objective value : " << setprecision(3) << solution.objective_value << endl;

    // Solve the integer version of the problem
    auto start_integer = chrono::steady_clock::now();
    IntegerSolution integer_solution = integer_RMP(instance, routes);
    auto end_integer = chrono::steady_clock::now();
    int integer_time = chrono::duration_cast<chrono::milliseconds>(end_integer - start_integer).count();

    cout << "Integer solution found with objective value : " << integer_solution.objective_value << endl;

    if (VERBOSE){
        // Print the number of times each vehicle's sub problem reached the time limit
        for (int i = 0; i < instance.vehicles.size(); i++){
            if (time_limit_reached[i] > 0) {
                cout << "Vehicle " << i << " : the time limit was reached " << time_limit_reached[i] << " times" << endl;
            }
        }
    }

    // Print the time it took to solve the master problem
    cout << "Total time spent solving the master problem : " << master_time << " ms" << endl;
    cout << "Total time spent solving the pricing problems : " << pricing_time << " ms" << endl;
    cout << "Total time spent solving the integer problem : " << integer_time << " ms" << endl;
    cout << "-----------------------------------" << endl;

    // Print the routes in the integer solution (in detail)
    full_analysis(integer_solution, routes, instance);

    int remaining_time = TIME_LIMIT - (master_time + pricing_time + integer_time) / 1000;
    // Keep only the routes that are used in the integer solution
    vector<Route> used_routes = keep_used_routes(routes, integer_solution);
    // We can then solve the compact model with these imposed routings
    CompactSolution<int> compact_solution = compact_solver(instance, remaining_time, used_routes, SOLVER_MODE);
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



