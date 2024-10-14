#include "full_procedure.h"

#include "master_problem/master.h"
#include "master_problem/rmp_solver.h"
#include "master_problem/node.h"

#include "pricing_problem/subproblem.h"

#include "routes/route_optimizer.h"

#include "repair/repair.h"

#include "algorithms/column_generation.h"

#include "data_analysis/analysis.h"
#include "data_analysis/export.h"

#include <iostream>
#include <iomanip>

CGResult full_cg_procedure(const Instance & instance, const ColumnGenerationParameters& parameters) {
    using std::vector, std::string;
    using std::cout, std::endl, std::setprecision;
    namespace chrono = std::chrono;

    auto procedure_start = chrono::steady_clock::now();

    vector<Route> routes;
    cout << "Initializing the routes with an empty route" << endl;
    routes = vector<Route>();
    routes.push_back(EmptyRoute(instance.nodes.size()));

    int MAX_RESOURCES_DOMINANCE = instance.capacities_labels.size() + 1;
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
        return CGResult{};
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

    int elapsed_time = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - procedure_start).count();
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
    string output_filename = "../results/" + instance.name + "_" + date + ".json";
    double seconds = elapsed_time / 1000.0;
    export_solution(output_filename, instance, integer_solution, routes, seconds, parameters, result.objective_values, result.objective_time_points);

}