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
#define N_INTERVENTIONS 75
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

    // Get the DualSolution from the first RMP
    vector<Route> routes = {EmptyRoute(instance.nodes.size())};
    BPNode node = RootNode(routes);
    MasterSolution master_solution = relaxed_RMP(instance, routes, node);
    DualSolution dual_solution = master_solution.dual_solution;
    Vehicle vehicle = instance.vehicles[0];


    // auto start = chrono::steady_clock::now();
    // // Solve the pricing problem for the first vehicle
    // cout << "Solving to optimality using Pathwyse" << endl;
    // Route route = solve_pricing_problem(instance, vehicle, dual_solution, true, -1);

    // auto end = chrono::steady_clock::now();
    // auto duration = chrono::duration_cast<chrono::milliseconds>(end - start).count();
    // cout << "Time to solve the pricing problem using Pathwyse: " << duration << "ms" << endl;

    // // Print the route
    // print_route(route, instance);

    cout << "-----------------------------------" << endl;

    auto start_pulse = chrono::steady_clock::now();
    // Solve it using the pulse algorithm
    int delta = 10;
    int pool_size = 100;
    cout << "Solving to optimality using Pulse with delta = " << delta << endl;
    vector<Route> routes_pulse = solve_pricing_problem_pulse(instance, vehicle, dual_solution, delta, pool_size);

    auto end_pulse = chrono::steady_clock::now();
    auto duration_pulse = chrono::duration_cast<chrono::milliseconds>(end_pulse - start_pulse).count();
    cout << "Time to solve the pricing problem using Pulse: " << duration_pulse << "ms" << endl;
    cout << "Found " << routes_pulse.size() << " routes" << endl;

    // Print the route
    for (Route route_pulse : routes_pulse){
        print_route(route_pulse, instance);
    }

}
