#include "instance/parser.h"
#include "instance/preprocessing.h"

#include "master_problem/master.h"
#include "master_problem/master_solver.h"
#include "master_problem/node.h"

#include "pricing_problem/subproblem.h"

#include "routes/route_optimizer.h"

#include "repair/repair.h"

#include "algorithms/column_generation.h"
#include "algorithms/full_procedure.h"

#include "data_analysis/analysis.h"
#include "data_analysis/export.h"
#include "data_analysis/plot.h"

#include <matplot/matplot.h>

#include <memory>   
#include <iostream>
#include <iomanip>
#include <chrono>
#include <format>
#include <filesystem>

inline constexpr int TIME_LIMIT = 60;
inline constexpr bool VERBOSE = true;
inline constexpr bool EXPORT_SOLUTION = true;


int main(int argc, char *argv[]){

    using std::cout, std::endl;
    using std::setprecision, std::fixed;
    using std::vector, std::string, std::to_string, std::pair;
    using std::tuple, std::set;
    using std::unique_ptr;
    namespace chrono = std::chrono;    

    int MAX_RESOURCES_DOMINANCE = -1;
    // Create the parameters for the column generation algorithm
    ColumnGenerationParameters parameters = ColumnGenerationParameters({
        {"time_limit", TIME_LIMIT},
        {"reduced_cost_threshold", 1e-6},
        {"verbose", true},
        {"max_iterations", 1000},
        {"max_consecutive_non_improvement", 5},
        {"compute_integer_solution", true},
        {"compute_intermediate_integer_solutions", false},
        {"use_maximisation_formulation", false},
        {"use_duration_only", true},
        {"max_resources_dominance", MAX_RESOURCES_DOMINANCE},
        {"ng", NG_STANDARD},
        {"dssr", DSSR_STANDARD},
        {"use_visited", true},
        {"bidirectional_DP", false},
        {"pathwyse_time_limit", 0.0},
        {"switch_to_cyclic_price", true},
        {"delta ", 50},
        {"solution_pool_size", 10},
        {"alpha", 0.5},
        {"use_stabilisation", false},
        {"pricing_function", PRICING_PATHWYSE_BASIC},
        {"pricing_verbose", false}
    });
    // -------------- Instance parsing --------------
    const std::string name = "agency1_19-01-2023_anonymized";
    string filename = "../data/" + name + ".json";
    Instance instance = parse_file(filename, name, 75, 20, false);
    // -------------- Preprocessing --------------
    preprocess_interventions(instance);
    auto max_duration = max_a_priori_feasible_time(instance, true);

    // -------------- Column generation --------------
    vector<Route> routes = {};
    routes.push_back(EmptyRoute(instance.nodes.size()));
    cout << "Starting the column generation algorithm" << endl;

    CGResult result = full_cg_procedure(instance, routes, parameters);

    return 0;
}
