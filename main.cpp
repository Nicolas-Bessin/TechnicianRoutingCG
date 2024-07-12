#include "src/parser.h"
#include "src/preprocessing.h"
#include "src/cg_solver.h"
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
    //cg_solver(instance, 1000);
}

