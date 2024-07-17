#include "src/parser.h"
#include "src/preprocessing.h"
#include "src/master.h"
#include "src/pricing.h"
#include <iostream>
#include "pathwyse/core/solver.h"
#include <chrono>


using namespace std;

int main(int, char**){
    std::cout << "Technician Routing Problem using Column Generation" << std::endl;
    string default_filename = "../data/instance_1.json";
    Instance instance = parse_file(default_filename);
    preprocess_interventions(instance);

    // Create the pricing sub problems for each vehicle
    vector<Problem*> pricing_problems;
    for (auto vehicle : instance.vehicles){
        Problem* pricing_problem = create_pricing_instance(instance, vehicle);
        pricing_problems.push_back(pricing_problem);
    }


    // Create a dummy route for the first vehicle
    vector<Route> routes;
    routes.push_back(Route(0, instance.number_interventions));

    // Count the time solving the master problem, and the pricing sub problems
    int master_time = 0;
    int pricing_time = 0;

    // Global time limit for the column generation algorithm of 60 seconds
    int time_limit = 10 * 1000;

    // Count the number of time each vehicle's sub problem reached the time limit
    vector<int> time_limit_reached(instance.vehicles.size(), 0);

    // Main loop of the column generation algorithm
    int iteration = 0;
    bool stop = false;
    MasterSolution solution;
    while (!stop && master_time + pricing_time < time_limit){
        // Solve the master problem
        auto start = chrono::high_resolution_clock::now();
        solution = cg_solver(instance, routes, 60);
        auto end = chrono::high_resolution_clock::now();
        int diff = chrono::duration_cast<chrono::milliseconds>(end - start).count();
        std::cout << "Master problem solved in " << diff << " ms" << std::endl;
        master_time += diff;
        // Solve each pricing sub problem
        auto start_pricing = chrono::high_resolution_clock::now();
        bool is_any_route_added = false;
        for (int v = 0; v < instance.vehicles.size(); v++){
            Problem* pricing_problem = pricing_problems.at(v);
            Vehicle vehicle = instance.vehicles.at(v);
            update_pricing_instance(pricing_problem, solution.alphas, solution.betas.at(vehicle.id), instance, vehicle);
            vector<Route> best_new_routes = solve_pricing_problem(pricing_problem, 5, instance, vehicle);
            if (best_new_routes.size() == 0){
                time_limit_reached[vehicle.id]++;
            }
            //std::cout << "Vehicle " << vehicle.id << " : " << best_new_routes.size() << " routes found" << std::endl;
            // Go through the returned routes, and add them to the master problem if they have a positive reduced cost
            for (auto route : best_new_routes){
                if (route.reduced_cost > 0){
                    //std::cout << "Route added with reduced cost " << route.reduced_cost << std::endl;
                    routes.push_back(route);
                    is_any_route_added = true;
                }
            }
        }
        auto end_pricing = chrono::high_resolution_clock::now();
        int diff_pricing = chrono::duration_cast<chrono::milliseconds>(end_pricing - start_pricing).count();
        std::cout << "Pricing sub problems solved in " << diff_pricing << " ms" << std::endl;
        pricing_time += diff_pricing;
        std::cout << "Iteration " << iteration << " - Objective value : " << solution.objective_value << std::endl;
        // If no route was added, we stop the algorithm
        if (!is_any_route_added){
            stop = true;
        }
        iteration++;
    }

    std::cout << "-----------------------------------" << std::endl;
    if (stop) {
        std::cout << "Found no new route to add" << std::endl;
    }
    std::cout << "End of the column generation after " << iteration << " iterations" << std::endl;
    std::cout << "Objective value : " << solution.objective_value << std::endl;

    // Solve the integer version of the problem
    auto start_integer = chrono::high_resolution_clock::now();
    IntegerSolution integer_solution = solve_integer_problem(instance, routes, 60);
    auto end_integer = chrono::high_resolution_clock::now();
    int diff_integer = chrono::duration_cast<chrono::milliseconds>(end_integer - start_integer).count();

    std::cout << "Integer solution found with objective value : " << integer_solution.objective_value << std::endl;

    std::cout << "-----------------------------------" << std::endl;
    std::cout << "Total time spent solving the master problem : " << master_time << " ms" << std::endl;
    std::cout << "Total time spent solving the pricing sub problems : " << pricing_time << " ms - average time : " << pricing_time / iteration << " ms" << std::endl;
    std::cout << "Total time spent solving the integer problem : " << diff_integer << " ms" << std::endl;

    std::cout << "Total running time : " << master_time + pricing_time + diff_integer << " ms" << std::endl;

    std::cout << "-----------------------------------" << std::endl;

    // Print the number of times each vehicle's sub problem reached the time limit
    for (int i = 0; i < instance.vehicles.size(); i++){
        if (time_limit_reached[i] > 0) {
            std::cout << "Vehicle " << i << " : the time limit was reached " << time_limit_reached[i] << " times" << std::endl;
        }
    }

    return 0;
}








// Dead code : Dump some info on the interventions
// for (int i = 0; i < instance.number_interventions; i++){
//     std::cout << "Intervention " << instance.nodes[i].id << " : " << instance.nodes[i].duration << " minutes between ";
//     std::cout << instance.nodes[i].start_window << " - " << instance.nodes[i].end_window;
//     std::cout << " - Is ambiguous : " << instance.nodes[i].is_ambiguous << " - Is long : " << instance.nodes[i].is_long;
//     std::cout << " - Number of vehicles : " << instance.nodes[i].nb_vehicles << std::endl;
// }
// // Dump some info on the vehicles
// for (auto vehicle : instance.vehicles){
//     std::cout << "Vehicle " << vehicle.id << " : ";
//     std::cout << "Number of interventions : " << vehicle.interventions.size() << " - ";
//     std::cout << "Depot : " << vehicle.depot << std::endl;
// }
// // Enumerate the interventions that can be performed by the first vehicle
// for (auto inter_index : instance.vehicles.at(0).interventions){
//     auto intervention = &(instance.nodes[inter_index]);
//     std::cout << "Intervention " << intervention->id << " : " << intervention->duration << " minutes between ";
//     std::cout << intervention->start_window << " - " << intervention->end_window;
//     std::cout << " - Is ambiguous : " << intervention->is_ambiguous << " - Is long : " << intervention->is_long;
//     std::cout << " - Number of vehicles : " << intervention->nb_vehicles << std::endl;
// }