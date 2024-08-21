#include "src/parser.h"
#include "src/preprocessing.h"
#include "src/plot.h"

#include "src/master.h"
#include "src/RMP_solver.h"
#include "src/pricing.h"

#include "src/column_generation.h"

#include "src/branch_and_price.h"

#include "src/analysis.h"

#include "src/compact_solution.h"
#include "src/compact_solver.h"
#include "src/solution_converter.h"

#include "src/heuristics.h"

#include <memory>   
#include <iostream>
#include <iomanip>
#include <chrono>

#define TIME_LIMIT 120
#define SOLVER_MODE IMPOSE_ROUTING
#define THRESHOLD 1e-6
#define VERBOSE false
#define GREEDY_INIT false

int main(int argc, char *argv[]){

    using std::cout, std::endl;
    using std::setprecision, std::fixed;
    using std::vector, std::string, std::to_string, std::pair;
    using std::tuple, std::set;
    using std::unique_ptr;
    namespace chrono = std::chrono;

    // Parse the instance from a JSON file
    auto start_parse = chrono::steady_clock::now();
    cout << "Technician Routing Problem using Column Generation" << endl;
    cout << "-----------------------------------" << endl;
    string default_filename = "../data/agency1_19-01-2023_anonymized.json";
    Instance instance = parse_file(default_filename, true);

    // Plot the instance
    plot_instance(instance);

    preprocess_interventions(instance);

    auto end_parse = chrono::steady_clock::now();
    int diff_parse = chrono::duration_cast<chrono::milliseconds>(end_parse - start_parse).count();

    cout << "Total time spent parsing the instance : " << diff_parse << " ms" << endl;

    vector<Route> routes = parse_routes_from_file("../routes/predefined_14_08.json", instance);

    cout << "-----------------------------------" << endl;
    // Get an integer solution from the routes
    BPNode root = RootNode(routes);
    IntegerSolution integer_solution = integer_RMP(instance, routes, root);
    MasterSolution master_solution = relaxed_RMP(instance, routes, root);

    print_used_routes(integer_solution, routes, instance);

    full_analysis(integer_solution, routes, instance);

    cout << "True cost of the integer solution : " << setprecision(10) <<  compute_integer_objective(integer_solution, routes, instance) << endl;
    cout << "True cost of the master solution : " << setprecision(10) <<  master_solution.objective_value << endl;

    // cout << "-----------------------------------" << endl;
    // // Convert those routes to a set a required edges
    // set<tuple<int, int, int>> required_edges = routes_to_required_edges(routes);
    // set<tuple<int, int, int>> forbidden_edges = set<tuple<int, int, int>>();
    // vector<int> order = {14, 3, 8, 7, 19, 6, 10, 0, 1, 2};

    // vector<Route> regenerated_routes = greedy_heuristic(instance, order, forbidden_edges, required_edges);
    // BPNode regenerated_root = RootNode(regenerated_routes);
    // IntegerSolution regenerated_solution = integer_RMP(instance, regenerated_routes, regenerated_root);

    // print_used_routes(regenerated_solution, regenerated_routes, instance);

    // full_analysis(regenerated_solution, regenerated_routes, instance);

}
