#include "src/parser.h"
#include "src/preprocessing.h"

#include "src/master.h"
#include "src/pricing.h"

#include "src/column_generation.h"

#include "src/branch_and_price.h"

#include "src/analysis.h"

#include "src/heuristics.h"

#include <memory>   
#include <iostream>
#include <iomanip>
#include <chrono>

#define TIME_LIMIT 60
#define SOLVER_MODE IMPOSE_ROUTING
#define THRESHOLD 1e-6
#define VERBOSE true
#define GREEDY_INIT false
#define CYCLIC_PRICING true
#define MAX_ITER 10000
#define MAX_DEPTH 10
#define COMPUTE_INTEGER_SOL true


int main(int argc, char *argv[]){

    using std::cout, std::endl;
    using std::setprecision, std::fixed;
    using std::vector, std::string, std::to_string, std::pair;
    using std::unique_ptr;
    namespace chrono = std::chrono;

    // Parse the instance from a JSON file
    auto start_parse = chrono::steady_clock::now();
    cout << "Technician Routing Problem using Column Generation" << endl;
    cout << "-----------------------------------" << endl;
    string default_filename = "../data/instance_1.json";
    Instance instance = parse_file(default_filename, true);

    // Only keep the first 20 nodes
    vector<int> kept_nodes = vector<int>(instance.number_interventions);
    for (int i = 0; i < 25; i++){
        kept_nodes[i] = 1;
    }
    instance = cut_instance(instance, kept_nodes);

    preprocess_interventions(instance);

    auto end_parse = chrono::steady_clock::now();
    int diff_parse = chrono::duration_cast<chrono::milliseconds>(end_parse - start_parse).count();

    cout << "Total time spent parsing the instance : " << diff_parse << " ms" << endl;

    cout << "-----------------------------------" << endl;
    vector<Route> routes;
    if (GREEDY_INIT) {
        cout << "Initializing the routes with a greedy heuristic" << endl;
        routes = greedy_heuristic(instance);
        IntegerSolution greedy_solution = IntegerSolution(vector<int>(routes.size(), 1), 0);
        greedy_solution.objective_value = compute_integer_objective(greedy_solution, routes, instance);
        cout << "Objective value of the greedy heuristic : " << greedy_solution.objective_value << endl;
    } else {
        cout << "Initializing the routes with an empty route" << endl;
        routes = vector<Route>();
        routes.push_back(EmptyRoute(instance.nodes.size()));
    }

    cout << "-----------------------------------" << endl;

    int MAX_RES_DOMINANCE = instance.capacities_labels.size() + 1;

    branch_and_price(
        instance,
        routes,
        MAX_RES_DOMINANCE,
        CYCLIC_PRICING,
        TIME_LIMIT,
        MAX_DEPTH,
        VERBOSE
        );

    return 0;
}
