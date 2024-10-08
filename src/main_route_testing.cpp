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
inline constexpr int N_INTERVENTIONS = 50;

inline constexpr int TIME_LIMIT = 1200;
inline constexpr bool VERBOSE = true;

inline constexpr int DELTA = 10;

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

    cout << "-----------------------------------" << endl;

    auto vehicle_groups = regroup_vehicles_by_depot(instance.vehicles);
    cout << "Delta = " << DELTA << endl;

    vector<int> vehicle_order = {};
    for (int v = 0; v < instance.number_vehicles; v++){
        if (instance.vehicles[v].interventions.size() > 0){
            vehicle_order.push_back(v);
        }
    }

    cout << "-----------------------------------" << endl;
    cout << "Solving the pricing problems sequentially" << endl;

    auto begin = chrono::steady_clock::now();
    vector<Route> new_routes_naive;
    for (auto vehicle : instance.vehicles) {
        if (vehicle.interventions.size() == 0) continue;
        auto new_routes_v = solve_pricing_problem_pulse(instance, vehicle, dual_solution, DELTA, 10);
        new_routes_naive.insert(new_routes_naive.end(), new_routes_v.begin(), new_routes_v.end());
    }

    auto end = chrono::steady_clock::now();
    int diff = chrono::duration_cast<chrono::milliseconds>(end - begin).count();
    cout << "Time spent solving the pricing problems in parallel : " << diff << " ms" << endl;

    cout << "-----------------------------------" << endl;
    cout << "Solving each pricing problem sequentially with the parallel PA" << endl;
    
    begin = chrono::steady_clock::now();

    vector<Route> new_routes_individually = full_pricing_problems_multithreaded_pulse(dual_solution, instance, vehicle_order, DELTA, 1);
    end = chrono::steady_clock::now();
    diff = chrono::duration_cast<chrono::milliseconds>(end - begin).count();
    cout << "Time spent solving the pricing problems sequentially with the parallel PA : " << diff << " ms" << endl;

}
