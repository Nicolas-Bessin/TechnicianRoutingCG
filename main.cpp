#include "src/parser.h"
#include "src/preprocessing.h"
#include "src/master.h"
#include "src/pricing.h"
#include <iostream>
#include "pathwyse/core/solver.h"

//using namespace std;

int main(int, char**){
    cout << "Technician Routing Problem using Column Generation" << endl;
    string default_filename = "../data/instance_1.json";
    Instance instance = parse_file(default_filename);
    preprocess_interventions(instance);


    // Create a dummy route for the first vehicle
    vector<Route> routes;
    routes.push_back(Route(0, instance.number_interventions));

    // Main loop of the column generation algorithm
    int iteration = 0;
    bool stop = false;
    MasterSolution solution;
    while (!stop){
        // Solve the master problem
        solution = cg_solver(instance, routes, 60);
        // Solve each pricing sub problem
        bool is_any_route_added = false;
        for (auto vehicle : instance.vehicles){
            Problem pricing_problem = create_pricing_instance(instance, vehicle);
            update_pricing_instance(pricing_problem, solution.alphas, solution.betas.at(vehicle.id), instance, vehicle);
            vector<Route> best_new_routes = solve_pricing_problem(pricing_problem, 5, instance, vehicle);
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
        if (!is_any_route_added || iteration > 3){
            stop = true;
        }
        cout << "Iteration " << iteration << " - Objective value : " << solution.objective_value << endl;
        iteration++;
    }

    cout << "End of the algorithm after " << iteration << " iterations" << endl;
    cout << "Objective value : " << solution.objective_value << endl;

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