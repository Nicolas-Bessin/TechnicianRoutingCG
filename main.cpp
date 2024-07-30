#include "src/parser.h"
#include "src/preprocessing.h"
#include "src/master.h"
#include "src/pricing.h"
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
#define THRESHOLD 1e-6


void takes_an_int(int a){
    std::cout << "Input was : " << a << std::endl;
}


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

    auto start_sub_building = chrono::steady_clock::now();
    // Create the pricing sub problems for each vehicle
    const string pricing_folder = "../pricing_instances/";
    vector<unique_ptr<Problem>> pricing_problems;
    for (auto vehicle : instance.vehicles){
        string filename = pricing_folder + "v_" + to_string(vehicle.id) + ".txt";
        //write_pricing_instance(filename, instance, vehicle);
        pricing_problems.push_back(create_pricing_instance(instance, vehicle, SCALE_FACTOR));
    }
    auto end_sub_building = chrono::steady_clock::now();
    int diff_sub_building = chrono::duration_cast<chrono::milliseconds>(end_sub_building - start_sub_building).count();


    // Create a dummy route for the first vehicle
    vector<Route> routes;
    routes.push_back(Route(0, instance.number_interventions));

    // Count the time solving the master problem, and the pricing sub problems
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
        solution = cg_solver(instance, routes, 60);
        auto end = chrono::steady_clock::now();
        int diff = chrono::duration_cast<chrono::milliseconds>(end - start).count();
        cout << "Master problem solved in " << diff << " ms \n";
        cout << "Number of interventions covered : " << setprecision(2) << count_covered_interventions(solution, routes, instance);
        cout << " - Number of zeros in alphas : " << count_zeros(solution.alphas) << " / " << solution.alphas.size() << endl;
        master_time += diff;

        // Solve each pricing sub problem
        auto start_pricing = chrono::steady_clock::now();
        int n_added_routes = 0;

        double max_reduced_cost = 0;


        for (int v = 0; v < instance.vehicles.size(); v++){
            const Vehicle& vehicle = instance.vehicles.at(v);
            update_pricing_instance(pricing_problems.at(v), solution.alphas, solution.betas[v], instance, vehicle, SCALE_FACTOR);
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
        cout << "Pricing sub problems solved in " << diff_pricing << " ms - Added " << n_added_routes << " routes";
        cout << " - Max reduced cost : " << setprecision(15) << max_reduced_cost / SCALE_FACTOR << "\n";
        pricing_time += diff_pricing;
        cout << "Iteration " << iteration << " - Objective value : " << solution.objective_value << "\n";
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
    IntegerSolution integer_solution = solve_integer_problem(instance, routes);
    auto end_integer = chrono::steady_clock::now();
    int diff_integer = chrono::duration_cast<chrono::milliseconds>(end_integer - start_integer).count();

    cout << "Integer solution found with objective value : " << integer_solution.objective_value << endl;
    cout << "Manual computing of the value : " << compute_integer_objective(integer_solution, routes, instance) << endl;

    cout << "-----------------------------------" << endl;
    cout << "Total time spent parsing the instance : " << diff_parse << " ms" << endl;
    cout << "Total time spent building the pricing sub problems : " << diff_sub_building << " ms" << endl;
    cout << "Total time spent solving the master problem : " << master_time << " ms" << endl;
    cout << "Total time spent solving the pricing sub problems : " << pricing_time << " ms - average time : " << pricing_time / iteration << " ms" << endl;
    cout << "Total time spent solving the integer problem : " << diff_integer << " ms" << endl;

    cout << "Total running time : " << master_time + pricing_time + diff_integer << " ms" << endl;

    cout << "-----------------------------------" << endl;

    // Print the routes in the integer solution (in detail)
    print_used_routes(integer_solution, routes, instance);

    // Print the number of times each vehicle's sub problem reached the time limit
    for (int i = 0; i < instance.vehicles.size(); i++){
        if (time_limit_reached[i] > 0) {
            cout << "Vehicle " << i << " : the time limit was reached " << time_limit_reached[i] << " times" << endl;
        }
    }
    int remaining_time = TIME_LIMIT - (master_time + pricing_time + diff_integer) / 1000;
    // Finally, we construct a vector of pairs (vehicle_id, intervention_id) that we can use to impose a routing
    vector<pair<int, int>> imposed_routings = imposed_routings_from_routes(routes, integer_solution);
    // We can then solve the compact model with these imposed routings
    CompactSolution compact_solution = compact_solver(instance, remaining_time, imposed_routings);
    // Convert back to routes
    vector<Route> compact_routes = compact_solution_to_routes(instance, compact_solution);
    // Create a dummy integer solution (all variables set to 1)
    IntegerSolution compact_integer_solution = IntegerSolution(vector<int>(compact_routes.size(), 1), compact_solution.objective_value);

    cout << "Manual computing of the compact solution value : " << compute_integer_objective(compact_integer_solution, compact_routes, instance) << endl;

    cout << "-----------------------------------" << endl;
    // Print the routes in the compact solution
    print_used_routes(compact_integer_solution, compact_routes, instance);

    // Run the analysis on the compact solution
    //full_analysis(compact_integer_solution, compact_routes, instance);

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



