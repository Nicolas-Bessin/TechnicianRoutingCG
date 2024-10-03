#include "instance/parser.h"
#include "instance/preprocessing.h"

#include "master_problem/master.h"
#include "pricing_problem/subproblem.h"

#include "algorithms/branch_and_price.h"

#include "data_analysis/analysis.h"

#include "algorithms/heuristics.h"
#include "algorithms/parameters.h"

#include <memory>   
#include <iostream>
#include <iomanip>
#include <chrono>

inline constexpr std::string INSTANCE_FILE = "instance_1";
inline constexpr int N_INTERVENTIONS = 25;

inline constexpr int TIME_LIMIT = 1200;
inline constexpr double THRESHOLD = 1e-6;
inline constexpr bool VERBOSE = true;
inline constexpr int MAX_ITER = 10000;
inline constexpr bool COMPUTE_INTEGER_SOL = true;

inline constexpr bool SWITCH_CYCLIC_PRICING = true;

inline constexpr int DELTA = 10;
inline constexpr int SOLUTION_POOL_SIZE = 10;

inline constexpr double ALPHA = 0.5;

inline constexpr int MAX_DEPTH = 10;


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
    string fileprefix = INSTANCE_FILE;
    string filename = "../data/" + fileprefix + ".json";
    Instance instance = parse_file(filename, fileprefix, N_INTERVENTIONS, VERBOSE);
    instance.M = compute_M_naive(instance);

    preprocess_interventions(instance);

    auto end_parse = chrono::steady_clock::now();
    int diff_parse = chrono::duration_cast<chrono::milliseconds>(end_parse - start_parse).count();

    cout << "Total time spent parsing the instance : " << diff_parse << " ms" << endl;

    cout << "-----------------------------------" << endl;
    vector<Route> routes;
    cout << "Initializing the routes with an empty route" << endl;
    routes = vector<Route>();
    routes.push_back(EmptyRoute(instance.nodes.size()));
    

    cout << "-----------------------------------" << endl;

    int MAX_RES_DOMINANCE = instance.capacities_labels.size() + 1;

    BranchAndPriceParameters parameters = BranchAndPriceParameters({
        {"time_limit", TIME_LIMIT},
        {"reduced_cost_threshold", THRESHOLD},
        {"verbose", VERBOSE},
        {"max_iterations", MAX_ITER},
        {"compute_integer_solution", COMPUTE_INTEGER_SOL},
        {"max_resources_dominance", MAX_RES_DOMINANCE},
        {"switch_to_cyclic_pricing", SWITCH_CYCLIC_PRICING},
        {"delta", DELTA},
        {"solution_pool_size", SOLUTION_POOL_SIZE},
        {"alpha", ALPHA},
        {"max_depth", MAX_DEPTH}
    }
    );

    branch_and_price(
        instance,
        routes,
        parameters
        );

    return 0;
}
