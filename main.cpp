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

    MasterSolution solution = cg_solver(instance, 1000);
    cout << "Cost of vehicle 0 : " << instance.vehicles.at(0).cost << endl;
    cout << "----------------------------------------------" << endl;
    // Define the sub problem for the first vehicle
    Problem princing0 = create_pricing_instance(instance, instance.vehicles.at(0));
    // Update it with the dual values
    update_pricing_instance(princing0, solution.alphas, solution.betas.at(0), instance, instance.vehicles.at(0));
    // Solve the sub problem
    vector<Route> routes0 = solve_pricing_problem(princing0, 5, instance, instance.vehicles.at(0));
    // Print the reduced cost of the routes
    for (int i = 0; i < routes0.size(); i++){
        cout << "Reduced cost of route " << i << " : " << routes0.at(i).reduced_cost << endl;
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