#include "instance/parser.h"
#include "instance/preprocessing.h"


#include "master_problem/master.h"
#include "master_problem/rmp_solver.h"

#include "pricing_problem/pricing.h"

#include "data_analysis/analysis.h"
#include "data_analysis/plot.h"

#include "algorithms/heuristics.h"

#include <memory>   
#include <iostream>
#include <iomanip>
#include <chrono>

#define TIME_LIMIT 120
#define SOLVER_MODE IMPOSE_ROUTING
#define THRESHOLD 1e-6
#define VERBOSE false
#define GREEDY_INIT false
#define N_INTERVENTIONS 25

int main(int argc, char *argv[]){

    using std::cout, std::endl;
    using std::setprecision, std::fixed;
    using std::vector, std::string, std::to_string, std::pair;
    using std::tuple, std::set;
    using std::unique_ptr;
    namespace chrono = std::chrono;

    // Test the random cycle generation
    vector<int> cycle = random_cycle(10);
    // Print the antecedents
    for (int i = 0; i < 10; i++){
        cout << i << " ";
    }
    cout << endl;
    // Print the cycle
    for (int i = 0; i < 10; i++){
        cout << cycle[i] << " ";
    }


    // // Parse the instance from a JSON file
    // auto start_parse = chrono::steady_clock::now();
    // cout << "Technician Routing Problem using Column Generation" << endl;
    // cout << "-----------------------------------" << endl;
    // string default_filename = "../data/agency1_19-01-2023_anonymized.json";
    // Instance instance = parse_file(default_filename, true);

    // //plot_instance(instance);

    // // // Only keep the first 25 nodes
    // // vector<int> kept_nodes = vector<int>(instance.number_interventions);
    // // for (int i = 0; i < N_INTERVENTIONS; i++){
    // //     kept_nodes[i] = 1;
    // // }
    // // instance = cut_instance(instance, kept_nodes);

    // // Plot the instance
    // // plot_instance(instance);

    // preprocess_interventions(instance);

    // auto end_parse = chrono::steady_clock::now();
    // int diff_parse = chrono::duration_cast<chrono::milliseconds>(end_parse - start_parse).count();

    // cout << "Total time spent parsing the instance : " << diff_parse << " ms" << endl;

    // vector<Route> routes = parse_routes_from_file("../routes/best_small.json", instance);

    // cout << "-----------------------------------" << endl;
    // // Get an integer solution from the routes
    // BPNode root = RootNode(routes);
    // IntegerSolution integer_solution = integer_RMP(instance, routes, root);
    // MasterSolution master_solution = relaxed_RMP(instance, routes, root);

    // print_used_routes(integer_solution, routes, instance);

    // full_analysis(integer_solution, routes, instance);

    // cout << "True cost of the integer solution : " << setprecision(10) <<  compute_integer_objective(integer_solution, routes, instance) << endl;
    // cout << "True cost of the master solution : " << setprecision(10) <<  master_solution.objective_value << endl;

    // // Plot the routes
    // plot_instance(instance, routes);

    // // cout << "-----------------------------------" << endl;
    // // // Convert those routes to a set a required edges
    // // set<tuple<int, int, int>> required_edges = routes_to_required_edges(routes);
    // // set<tuple<int, int, int>> forbidden_edges = set<tuple<int, int, int>>();
    // // vector<int> order = {14, 3, 8, 7, 19, 6, 10, 0, 1, 2};

    // // vector<Route> regenerated_routes = greedy_heuristic(instance, order, forbidden_edges, required_edges);
    // // BPNode regenerated_root = RootNode(regenerated_routes);
    // // IntegerSolution regenerated_solution = integer_RMP(instance, regenerated_routes, regenerated_root);

    // // print_used_routes(regenerated_solution, regenerated_routes, instance);

    // // full_analysis(regenerated_solution, regenerated_routes, instance);

}
