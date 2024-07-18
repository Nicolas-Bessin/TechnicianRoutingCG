#include "src/parser.h"
#include "src/preprocessing.h"
#include "src/master.h"
#include "src/pricing.h"
#include "pathwyse/core/solver.h"

#include <memory>   
#include <iostream>
#include <chrono>

using std::cout, std::endl;
using std::vector, std::string;
using std::unique_ptr;
namespace chrono = std::chrono;

int main(int, char**){

    cout << "Technician Routing Problem using Column Generation" << endl;
    string default_filename = "../data/instance_1.json";
    Instance instance = parse_file(default_filename);
    preprocess_interventions(instance);

    // Create the pricing sub problems for each vehicle
    vector<unique_ptr<Problem>> pricing_problems;
    for (auto vehicle : instance.vehicles){
        pricing_problems.push_back(create_pricing_instance(instance, vehicle));
    }


    // Create a dummy route for the first vehicle
    vector<Route> routes;
    routes.push_back(Route(0, instance.number_interventions));

    // Count the time solving the master problem, and the pricing sub problems
    int master_time = 0;
    int pricing_time = 0;

    // Global time limit for the column generation algorithm of 60 seconds
    int time_limit = 15 * 1000;

    // Count the number of time each vehicle's sub problem reached the time limit
    vector<int> time_limit_reached(instance.vehicles.size(), 0);

    // Main loop of the column generation algorithm
    int iteration = 0;
    bool stop = false;
    MasterSolution solution;
    while (!stop && master_time + pricing_time < time_limit){
        // Solve the master problem
        auto start = chrono::steady_clock::now();
        solution = cg_solver(instance, routes, 60);
        auto end = chrono::steady_clock::now();
        int diff = chrono::duration_cast<chrono::milliseconds>(end - start).count();
        cout << "Master problem solved in " << diff << " ms \n";
        master_time += diff;
        // Solve each pricing sub problem
        auto start_pricing = chrono::steady_clock::now();
        int n_added_routes = 0;

        for (int v = 0; v < instance.vehicles.size(); v++){
            Vehicle vehicle = instance.vehicles.at(v);
            update_pricing_instance(pricing_problems.at(v), solution.alphas, solution.betas.at(vehicle.id), instance, vehicle);
            vector<Route> best_new_routes = solve_pricing_problem(pricing_problems.at(v), 5, instance, vehicle);
            if (best_new_routes.size() == 0){
                time_limit_reached[vehicle.id]++;
            }
            //cout << "Vehicle " << vehicle.id << " : " << best_new_routes.size() << " routes found" << endl;
            // Go through the returned routes, and add them to the master problem if they have a positive reduced cost
            for (const auto &route : best_new_routes){
                if (route.reduced_cost > 0){
                    //cout << "Route added with reduced cost " << route.reduced_cost << endl;
                    routes.push_back(route);
                    n_added_routes++;
                }
            }
        }
        auto end_pricing = chrono::steady_clock::now();
        int diff_pricing = chrono::duration_cast<chrono::milliseconds>(end_pricing - start_pricing).count();
        cout << "Pricing sub problems solved in " << diff_pricing << " ms - Added " << n_added_routes << " routes \n";
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
    cout << "Objective value : " << solution.objective_value << endl;

    // Solve the integer version of the problem
    auto start_integer = chrono::steady_clock::now();
    IntegerSolution integer_solution = solve_integer_problem(instance, routes, 60);
    auto end_integer = chrono::steady_clock::now();
    int diff_integer = chrono::duration_cast<chrono::milliseconds>(end_integer - start_integer).count();

    cout << "Integer solution found with objective value : " << integer_solution.objective_value << endl;

    cout << "-----------------------------------" << endl;
    cout << "Total time spent solving the master problem : " << master_time << " ms" << endl;
    cout << "Total time spent solving the pricing sub problems : " << pricing_time << " ms - average time : " << pricing_time / iteration << " ms" << endl;
    cout << "Total time spent solving the integer problem : " << diff_integer << " ms" << endl;

    cout << "Total running time : " << master_time + pricing_time + diff_integer << " ms" << endl;

    cout << "-----------------------------------" << endl;

    // Print the number of times each vehicle's sub problem reached the time limit
    for (int i = 0; i < instance.vehicles.size(); i++){
        if (time_limit_reached[i] > 0) {
            cout << "Vehicle " << i << " : the time limit was reached " << time_limit_reached[i] << " times" << endl;
        }
    }

    return 0;
}








// Dead code : Dump some info on the interventions
// for (int i = 0; i < instance.number_interventions; i++){
//     cout << "Intervention " << instance.nodes[i].id << " : " << instance.nodes[i].duration << " minutes between ";
//     cout << instance.nodes[i].start_window << " - " << instance.nodes[i].end_window;
//     cout << " - Is ambiguous : " << instance.nodes[i].is_ambiguous << " - Is long : " << instance.nodes[i].is_long;
//     cout << " - Number of vehicles : " << instance.nodes[i].nb_vehicles << endl;
// }
// // Dump some info on the vehicles
// for (auto vehicle : instance.vehicles){
//     cout << "Vehicle " << vehicle.id << " : ";
//     cout << "Number of interventions : " << vehicle.interventions.size() << " - ";
//     cout << "Depot : " << vehicle.depot << endl;
// }
// // Enumerate the interventions that can be performed by the first vehicle
// for (auto inter_index : instance.vehicles.at(0).interventions){
//     auto intervention = &(instance.nodes[inter_index]);
//     cout << "Intervention " << intervention->id << " : " << intervention->duration << " minutes between ";
//     cout << intervention->start_window << " - " << intervention->end_window;
//     cout << " - Is ambiguous : " << intervention->is_ambiguous << " - Is long : " << intervention->is_long;
//     cout << " - Number of vehicles : " << intervention->nb_vehicles << endl;
// }