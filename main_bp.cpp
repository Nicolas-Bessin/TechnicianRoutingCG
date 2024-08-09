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
    string default_filename = "../data/instance_1_all_feasible.json";
    Instance instance = parse_file(default_filename, true);

    // Check wether the time and distance matrices are symetric
    cout << "Distance matrix is symetric : " << is_symmetric(instance.distance_matrix) << " - Biggest gap : " << symmetry_gap(instance.distance_matrix) << endl;
    cout << "Time matrix is symetric : " << is_symmetric(instance.time_matrix) << " - Biggest gap : " << symmetry_gap(instance.time_matrix) << endl;

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
        routes.push_back(Route(instance.nodes.size()));
    }

    cout << "-----------------------------------" << endl;

    branch_and_price(
        instance,
        routes,
        THRESHOLD,
        TIME_LIMIT,
        10
    );

    return 0;
}
