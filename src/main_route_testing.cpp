#include "instance/parser.h"
#include "instance/preprocessing.h"

#include "clustering/clustering.h"


#include "master_problem/master.h"
#include "master_problem/rmp_solver.h"

#include "pricing_problem/pricing.h"
#include "pricing_problem/subproblem.h"

#include "data_analysis/analysis.h"
#include "data_analysis/plot.h"

#include "algorithms/heuristics.h"

#include <memory>   
#include <iostream>
#include <iomanip>
#include <chrono>
#include <random>

#define TIME_LIMIT 60
#define THRESHOLD 1e-6
#define VERBOSE true
#define N_INTERVENTIONS 10
#define INSTANCE_FILE "instance_1"

int main(int argc, char *argv[]){

    using std::cout, std::endl;
    using std::setprecision, std::fixed;
    using std::vector, std::string, std::to_string, std::pair;
    namespace chrono = std::chrono;

    // Parse the instance from a JSON file
    cout << "Technician Routing Problem using Column Generation" << endl;
    cout << "-----------------------------------" << endl;
    string fileprefix = INSTANCE_FILE;
    string filename = "../data/" + fileprefix + ".json";
    Instance instance = parse_file(filename, fileprefix, true);

    // Only keep the first N_INTERVENTIONS nodes
    vector<int> kept_nodes = vector<int>(instance.number_interventions);
    for (int i = 0; i < N_INTERVENTIONS; i++){
        kept_nodes[i] = 1;
    }
    instance = cut_instance(instance, kept_nodes);

    preprocess_interventions(instance);

    // Create a "fake" dual solution filled with zeros
    int n_inter = instance.number_interventions;
    int n_vehicles = instance.vehicles.size();
    DualSolution dual_solution = DualSolution{
        vector<double>(n_inter, 0.0),
        vector<double>(n_vehicles, 0.0),
    };

    // Solve the pricing problem for the first vehicle
    cout << "Solving the first pricing problem -- Pathwyse" << endl;
    Vehicle vehicle = instance.vehicles[0];
    Route route = solve_pricing_problem(instance, vehicle, dual_solution);

    // Print the route
    print_route(route, instance);

    // Solve it using the pulse algorithm
    cout << "Solving the first pricing problem -- Pulse" << endl;
    Route route_pulse = solve_pricing_problem_pulse(instance, vehicle, dual_solution);

    // Print the route
    print_route(route_pulse, instance);

}
