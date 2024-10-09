#include "instance/parser.h"
#include "instance/preprocessing.h"

#include "master_problem/master.h"
#include "master_problem/rmp_solver.h"
#include "master_problem/node.h"

#include "pricing_problem/subproblem.h"

#include "routes/route_optimizer.h"

#include "repair/repair.h"

#include "algorithms/column_generation.h"

#include "data_analysis/analysis.h"
#include "data_analysis/export.h"

#include <memory>   
#include <iostream>
#include <iomanip>
#include <chrono>
#include <format>

inline constexpr std::string INSTANCE_FILE = "instance_1";
inline constexpr int N_INTERVENTIONS = 25;

inline constexpr int TIME_LIMIT = 1200;
inline constexpr bool VERBOSE = true;


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
    cout << "Starting the column generation algorithm" << endl;
    

    int MAX_RESOURCES_DOMINANCE = instance.capacities_labels.size() + 1;
    // Create the parameters for the column generation algorithm
    ColumnGenerationParameters parameters = ColumnGenerationParameters({
        {"time_limit", TIME_LIMIT},
        {"reduced_cost_threshold", 1e-6},
        {"verbose", VERBOSE},
        {"max_iterations", 1000},
        {"max_consecutive_non_improvement", 5},
        {"compute_integer_solution", true},
        {"max_resources_dominance", MAX_RESOURCES_DOMINANCE},
        {"switch_to_cyclic_price", true},
        {"delta ", 50},
        {"solution_pool_size", 10},
        {"alpha", 0.5},
        {"use_stabilisation", false},
        {"pricing_function", PRICING_PULSE_GROUPED_PAR_PAR},
        {"pricing_verbose", false}
    });
    // Create a root node for the algorithm
    BPNode root = RootNode(routes);
    CGResult result = column_generation(
        instance,
        root, 
        routes, 
        parameters
        );

    // Extract the results from the column generation algorithm
    int master_time = result.master_time;
    int pricing_time = result.pricing_time;
    int integer_time = result.integer_time;

    MasterSolution master_solution = result.master_solution;
    IntegerSolution integer_solution = result.integer_solution;


    // If the integer solution is not feasible, we can't do much
    if (!integer_solution.is_feasible){
        cout << "-----------------------------------" << endl;
        cout << "The integer solution is not feasible" << endl;
        return 1;
    }

    int n_routes_generated = routes.size();
    cout << "Number of routes generated : " << n_routes_generated;
    cout << " - Number of duplicate routes : " << count_routes_with_duplicates(routes);
    cout << " - Average time to generate a route : " << pricing_time / n_routes_generated << " ms" << endl;
    // Repair the integer solution
    cout << "-----------------------------------" << endl;
    cout << "Repairing the integer solution" << endl;
    routes = repair_routes(routes, integer_solution, instance);
    integer_solution = AllOnesSolution(routes.size());
    integer_solution.objective_value = compute_integer_objective(integer_solution, routes, instance);
    cout << "Objective value of the repaired solution : " << setprecision(15) << integer_solution.objective_value << endl;

    // Print the routes in the integer solution (in detail)
    // full_analysis(integer_solution, routes, instance);

    int elapsed_time = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start_parse).count();
    // Print the time it took to solve the master problem
    cout << "-----------------------------------" << endl;
    cout << "Total time spent solving the master problem : " << master_time << " ms" << endl;
    cout << "Total time spent solving the pricing problems : " << pricing_time << " ms - Average : " << pricing_time / result.number_of_iterations << " ms" << endl;
    cout << "Total time spent solving the integer problem : " << integer_time << " ms" << endl;
    cout << "Total elapsed time : " << elapsed_time << " ms" << endl;

    full_analysis(integer_solution, routes, instance);
    cout << "True cost of the integer solution : " << compute_integer_objective(integer_solution, routes, instance) << endl;

    cout << "-----------------------------------" << endl;

    //print_used_routes(integer_solution, routes, instance);

    // Dump the results to a file
    // Append the time to the filename
    const auto now = chrono::zoned_time(std::chrono::current_zone(), chrono::system_clock::now());
    string date = std::format("{:%Y-%m-%d-%H-%M-%OS}", now);
    string output_filename = "../results/" + fileprefix + "_" + date + ".json";
    double seconds = elapsed_time / 1000.0;
    export_solution(output_filename, instance, integer_solution, routes, seconds, parameters);
    

    return 0;
}
