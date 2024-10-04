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


inline constexpr std::string INSTANCE_FILE = "instance_1";
inline constexpr int N_INTERVENTIONS = 75;

inline constexpr int TIME_LIMIT = 1200;
inline constexpr bool VERBOSE = true;

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

    cout << "-----------------------------------" << endl;

    auto vehicle_groups = regroup_vehicles_by_depot(instance.vehicles);

    cout << "-----------------------------------" << endl;
    cout << "Solving the pricing problems group by group" << endl;

    auto begin = chrono::steady_clock::now();
    vector<Route> new_routes_grouped;
    for (auto [depot, group] : vehicle_groups) {
        auto group_routes = solve_pricing_problem_pulse_grouped(instance, group, dual_solution, 10, 1);
        new_routes_grouped.insert(new_routes_grouped.end(), group_routes.begin(), group_routes.end());
    }

    // Print the routes
    cout << "Routes generated : " << endl;
    for (Route route : new_routes_grouped) {
        print_route_reduced(route, instance);
        cout << "------------" << endl;
    }

    auto end = chrono::steady_clock::now();
    int diff = chrono::duration_cast<chrono::milliseconds>(end - begin).count();
    cout << "Time spent solving the pricing problem group by group : " << diff << " ms" << endl;

    cout << "-----------------------------------" << endl;
    cout << "Solving each pricing problem sequentially" << endl;
     
    begin = chrono::steady_clock::now();

    vector<Route> new_routes_individually;
    for (int v = 0; v < instance.vehicles.size(); v++) {
        const Vehicle& vehicle = instance.vehicles[v];
        auto new_routes_v = solve_pricing_problem_pulse(instance, vehicle, dual_solution, 10, 1);
        new_routes_individually.insert(new_routes_individually.end(), new_routes_v.begin(), new_routes_v.end());
    }

    // Print the routes
    cout << "Routes generated : " << endl;
    for (Route route : new_routes_individually) {
        print_route_reduced(route, instance);
        cout << "------------" << endl;
    }

    end = chrono::steady_clock::now();
    diff = chrono::duration_cast<chrono::milliseconds>(end - begin).count();
    cout << "Time spent solving the pricing problems individually : " << diff << " ms" << endl;

}
