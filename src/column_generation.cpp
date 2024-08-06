#include "column_generation.h"
#include "master.h"
#include "analysis.h"

#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>


CGResult column_generation(const Instance & instance, std::vector<Route> initial_routes, double reduced_cost_threshold, int time_limit, bool verbose){
    using std::cout, std::endl;
    using std::setprecision, std::fixed;
    using std::vector, std::string, std::to_string;
    using std::unique_ptr;
    namespace chrono = std::chrono;

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
    int building_time = chrono::duration_cast<chrono::milliseconds>(end_sub_building - start_sub_building).count();

    // Copy the routes to avoid modifying the input
    vector<Route> routes = initial_routes;

    int master_time = 0;
    int pricing_time = 0;

    // Count the number of time each vehicle's sub problem reached the time limit
    vector<int> time_limit_reached(instance.vehicles.size(), 0);
    // Aslo count the number of routes added for each vehicle
    vector<int> n_routes_per_v(instance.vehicles.size(), 0);

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
        if (verbose) {
            cout << "Master problem solved in " << diff << " ms \n";
            cout << "Number of interventions covered : " << setprecision(2) << count_covered_interventions(solution, routes, instance);
            cout << " - Number of vehicles used : " << count_used_vehicles(solution, routes, instance) << "\n";
            cout << "Min alpha : " << *std::min_element(solution.alphas.begin(), solution.alphas.end());
            cout << " - Max alpha : " << *std::max_element(solution.alphas.begin(), solution.alphas.end());
            cout << " - Min beta : " << *std::min_element(solution.betas.begin(), solution.betas.end());
            cout << " - Max beta : " << *std::max_element(solution.betas.begin(), solution.betas.end()) << "\n";
        }
        master_time += diff;

        // Solve each pricing sub problem
        auto start_pricing = chrono::steady_clock::now();
        int n_added_routes = 0;

        double max_reduced_cost = 0;

        // Explore all the vehicles in order at each iteration
        vector<int> vehicle_order(instance.vehicles.size());
        std::iota(vehicle_order.begin(), vehicle_order.end(), 0);

        for (const int& v : vehicle_order){
            const Vehicle& vehicle = instance.vehicles.at(v);
            update_pricing_instance(pricing_problems.at(v), solution.alphas, solution.betas[v], instance, vehicle);
            vector<Route> best_new_routes = solve_pricing_problem(pricing_problems.at(v), 5, instance, vehicle);
            if (best_new_routes.size() == 0){
                time_limit_reached[vehicle.id]++;
            }
            // Go through the returned routes, and add them to the master problem if they have a positive reduced cost
            for (const auto &route : best_new_routes){
                double reduced_cost = route.reduced_cost - solution.betas[v] - vehicle.cost;
                max_reduced_cost = std::max(max_reduced_cost, reduced_cost);
                //double computed_reduced_cost = compute_reduced_cost(route, solution.alphas, solution.betas[v], instance);
                if (reduced_cost> reduced_cost_threshold){
                    routes.push_back(route);
                    n_added_routes++;
                    n_routes_per_v[v]++;
                }
            }
        }

        auto end_pricing = chrono::steady_clock::now();
        int diff_pricing = chrono::duration_cast<chrono::milliseconds>(end_pricing - start_pricing).count();
        pricing_time += diff_pricing;
        if (verbose) {
            cout << "Pricing sub problems solved in " << diff_pricing << " ms - Added " << n_added_routes << " routes";
            cout << " - Max reduced cost : " << setprecision(15) << max_reduced_cost << "\n";
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
    cout << "Relaxed RMP objective value : " << setprecision(3) << solution.objective_value << endl;

    // Solve the integer version of the problem
    auto start_integer = chrono::steady_clock::now();
    IntegerSolution integer_solution = integer_RMP(instance, routes);
    auto end_integer = chrono::steady_clock::now();
    int diff_integer = chrono::duration_cast<chrono::milliseconds>(end_integer - start_integer).count();

    cout << "Integer RMP objective value : " << integer_solution.objective_value << endl;

    // Print the number of routes added for each vehicle
    cout << "Number of routes added for each vehicle : " << endl;
    for (int i = 0; i < instance.vehicles.size(); i++){
        cout << "v" << i << " : " << n_routes_per_v[i] << ", ";
    }
    cout << endl;

    if (verbose){
        // Print the number of times each vehicle's sub problem reached the time limit
        for (int i = 0; i < instance.vehicles.size(); i++){
            if (time_limit_reached[i] > 0) {
                cout << "Vehicle " << i << " : the time limit was reached " << time_limit_reached[i] << " times" << endl;
            }
        }
    }

    // Build the result object
    CGResult result;
    result.master_solution = solution;
    result.integer_solution = integer_solution;
    result.routes = routes;
    result.number_of_iterations = iteration;
    result.master_time = master_time;
    result.pricing_time = pricing_time;
    result.integer_time = diff_integer;
    result.building_time = building_time;

    return result;
}
