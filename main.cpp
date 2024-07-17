#include "src/parser.h"
#include "src/preprocessing.h"
#include "src/master.h"
#include "src/pricing.h"
#include <iostream>
#include "pathwyse/core/solver.h"
#include <chrono>


//using namespace std;

int main(int, char**){
    cout << "Technician Routing Problem using Column Generation" << endl;
    string default_filename = "../data/instance_1.json";
    Instance instance = parse_file(default_filename);
    preprocess_interventions(instance);


    // Create a dummy route for the first vehicle
    vector<Route> routes;
    routes.push_back(Route(0, instance.number_interventions));

    // Count the time solving the master problem, and the pricing sub problems
    int master_time = 0;
    int pricing_time = 0;

    // Count the number of time each vehicle's sub problem reached the time limit
    vector<int> time_limit_reached(instance.vehicles.size(), 0);

    // Main loop of the column generation algorithm
    int iteration = 0;
    bool stop = false;
    MasterSolution solution;
    while (!stop){
        // Solve the master problem
        auto start = chrono::high_resolution_clock::now();
        solution = cg_solver(instance, routes, 60);
        auto end = chrono::high_resolution_clock::now();
        int diff = chrono::duration_cast<chrono::milliseconds>(end - start).count();
        cout << "Master problem solved in " << diff << " ms" << endl;
        master_time += diff;
        // Solve each pricing sub problem
        auto start_pricing = chrono::high_resolution_clock::now();
        bool is_any_route_added = false;
        for (auto vehicle : instance.vehicles){
            Problem pricing_problem = create_pricing_instance(instance, vehicle);
            update_pricing_instance(pricing_problem, solution.alphas, solution.betas.at(vehicle.id), instance, vehicle);
            vector<Route> best_new_routes = solve_pricing_problem(pricing_problem, 5, instance, vehicle);
            if (best_new_routes.size() == 0){
                time_limit_reached[vehicle.id]++;
            }
            //cout << "Vehicle " << vehicle.id << " : " << best_new_routes.size() << " routes found" << endl;
            // Go through the returned routes, and add them to the master problem if they have a positive reduced cost
            for (auto route : best_new_routes){
                if (route.reduced_cost > 0){
                    //cout << "Route added with reduced cost " << route.reduced_cost << endl;
                    routes.push_back(route);
                    is_any_route_added = true;
                }
            }
        }
        // If no route was added, we stop the algorithm
        if (!is_any_route_added || iteration > 20){
            stop = true;
        }
        auto end_pricing = chrono::high_resolution_clock::now();
        int diff_pricing = chrono::duration_cast<chrono::milliseconds>(end_pricing - start_pricing).count();
        cout << "Pricing sub problems solved in " << diff_pricing << " ms" << endl;
        pricing_time += diff_pricing;
        cout << "Iteration " << iteration << " - Objective value : " << solution.objective_value << endl;
        iteration++;
    }

    cout << "End of the column generation after " << iteration << " iterations" << endl;
    cout << "Objective value : " << solution.objective_value << endl;

    // Solve the integer version of the problem
    auto start_integer = chrono::high_resolution_clock::now();
    IntegerSolution integer_solution = solve_integer_problem(instance, routes, 60);
    auto end_integer = chrono::high_resolution_clock::now();
    int diff_integer = chrono::duration_cast<chrono::milliseconds>(end_integer - start_integer).count();

    cout << "Integer solution found with objective value : " << integer_solution.objective_value << endl;
    cout << "Total time spent solving the master problem : " << master_time << " ms" << endl;
    cout << "Total time spent solving the pricing sub problems : " << pricing_time << " ms" << endl;
    cout << "Total time spent solving the integer problem : " << diff_integer << " ms" << endl;

    // Print the number of times each vehicle's sub problem reached the time limit
    for (int i = 0; i < instance.vehicles.size(); i++){
        cout << "Vehicle " << i << " : the time limit was reached " << time_limit_reached[i] << " times" << endl;
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