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

    // Dump some info on the interventions
    for (int i = 0; i < instance.number_interventions; i++){
        cout << "Intervention " << instance.nodes[i].id << " : " << instance.nodes[i].duration << " minutes between ";
        cout << instance.nodes[i].start_window << " - " << instance.nodes[i].end_window;
        cout << " - Is ambiguous : " << instance.nodes[i].is_ambiguous << " - Is long : " << instance.nodes[i].is_long;
        cout << " - Number of vehicles : " << instance.nodes[i].nb_vehicles << endl;

    }

    // Dump some info on the vehicles
    for (auto vehicle : instance.vehicles){
        cout << "Vehicle " << vehicle.id << " : ";
        cout << "Number of interventions : " << vehicle.interventions.size() << " - ";
        cout << "Depot : " << vehicle.depot->id << " \n";
    }

    // Enumerate the interventions that can be performed by the first vehicle
    for (auto intervention : instance.vehicles.at(0).interventions){
        cout << "Intervention " << intervention->id << " : " << intervention->duration << " minutes between ";
        cout << intervention->start_window << " - " << intervention->end_window;
        cout << " - Is ambiguous : " << intervention->is_ambiguous << " - Is long : " << intervention->is_long;
        cout << " - Number of vehicles : " << intervention->nb_vehicles << endl;
    }

    //cg_solver(instance, 1000);
    // Build a fake alphas vector with 49 zeros
    vector<double> alphas(49, 0);

    find_best_route(instance, instance.vehicles.at(0), alphas, 0.0);

    return 0;
}

