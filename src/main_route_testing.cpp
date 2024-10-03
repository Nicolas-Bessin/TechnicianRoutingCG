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


inline const std::string INSTANCE_FILE = "instance_1";
inline const int N_INTERVENTIONS = 25;

inline const int TIME_LIMIT = 1200;
inline const bool VERBOSE = true;

int main(int argc, char *argv[]){

    using std::cout, std::endl;
    using std::setprecision, std::fixed;
    using std::vector, std::string, std::to_string, std::pair;
    namespace chrono = std::chrono;

    // Parse the instance from a JSON file
    cout << "Technician Routing Problem - Testing" << endl;
    cout << "-----------------------------------" << endl;
    string fileprefix = INSTANCE_FILE;
    string filename = "../data/" + fileprefix + ".json";
    Instance instance = parse_file(filename, fileprefix, N_INTERVENTIONS, VERBOSE);
    instance.M = compute_M_naive(instance);

    preprocess_interventions(instance);

    // Get the DualSolution from the first RMP
    vector<Route> routes = {EmptyRoute(instance.nodes.size())};
    BPNode node = RootNode(routes);
    MasterSolution master_solution = relaxed_RMP(instance, routes, node);
    DualSolution dual_solution = master_solution.dual_solution;
    master_solution.dual_solution = dual_solution;
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
    int pool_size = 10;
    cout << "Solving to optimality using Pulse with delta = " << delta << endl;
    vector<Route> routes_pulse = solve_pricing_problem_pulse(instance, vehicle, dual_solution, delta, pool_size);

    auto end_pulse = chrono::steady_clock::now();
    auto duration_pulse = chrono::duration_cast<chrono::milliseconds>(end_pulse - start_pulse).count();
    cout << "Time to solve the pricing problem using Pulse: " << duration_pulse << "ms" << endl;
    cout << "Found " << routes_pulse.size() << " routes" << endl;
    cout << "Best reduced cost : " << routes_pulse[0].reduced_cost << endl;
    cout << "-----" << endl;

    Route best_route = routes_pulse[0];
    // Print the route along with the master solution
    print_route(best_route, instance, master_solution);

}
