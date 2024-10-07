#include "instance/parser.h"
#include "instance/preprocessing.h"

#include "clustering/clustering.h"

#include "master_problem/master.h"
#include "master_problem/rmp_solver.h"

#include "pricing_problem/full_pricing.h"
#include "pricing_problem/subproblem.h"

#include "data_analysis/analysis.h"
#include "data_analysis/plot.h"

#include "algorithms/heuristics.h"

#include <memory>   
#include <iostream>
#include <iomanip>
#include <chrono>
#include <random>


inline constexpr std::string INSTANCE_FILE = "instance_1";
inline constexpr int N_INTERVENTIONS = 10;

inline constexpr int TIME_LIMIT = 1200;
inline constexpr bool VERBOSE = true;

inline constexpr int DELTA = 50;

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

    // Solve the pricing problem for the first vehicle
    auto new_routes = solve_pricing_problem_pulse_parallel(instance, vehicle, dual_solution, DELTA, 1);

    // Print the routes
    for (const auto& route : new_routes){
        print_route_reduced(route, instance);
        cout << "-------" << endl;
    }

    // cout << "-----------------------------------" << endl;

    // auto vehicle_groups = regroup_vehicles_by_depot(instance.vehicles);
    // cout << "Delta = " << DELTA << endl;

    // cout << "-----------------------------------" << endl;
    // cout << "Solving the pricing problems group by group" << endl;

    // auto begin = chrono::steady_clock::now();
    // vector<Route> new_routes_grouped = full_pricing_problems_grouped_pulse(dual_solution, instance, vehicle_groups, DELTA, 1);

    // auto end = chrono::steady_clock::now();
    // int diff = chrono::duration_cast<chrono::milliseconds>(end - begin).count();
    // cout << "Time spent solving the pricing problem group by group : " << diff << " ms" << endl;

    // cout << "-----------------------------------" << endl;
    // cout << "Solving each pricing problem sequentially" << endl;
     
    // begin = chrono::steady_clock::now();

    // vector<int> vehicle_order = {};
    // for (int v = 0; v < instance.number_vehicles; v++){
    //     if (instance.vehicles[v].interventions.size() > 0){
    //         vehicle_order.push_back(v);
    //     }
    // }
    // vector<Route> new_routes_individually = full_pricing_problems_basic_pulse(dual_solution, instance, vehicle_order, DELTA, 1);

    // end = chrono::steady_clock::now();
    // diff = chrono::duration_cast<chrono::milliseconds>(end - begin).count();
    // cout << "Time spent solving the pricing problems individually : " << diff << " ms" << endl;

}
