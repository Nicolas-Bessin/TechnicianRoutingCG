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

CGResult full_cg_procedure(const Instance & instance, std::vector<Route>& routes, const ColumnGenerationParameters& parameters) {
    using std::vector, std::string;
    using std::cout, std::endl, std::setprecision;
    namespace chrono = std::chrono;

    auto procedure_start = chrono::steady_clock::now();

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

    // If the integer solution is not feasible, we can't do much
    if (!result.integer_solution.is_feasible){
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
    repair_routes(routes, result.integer_solution, instance);
    result.integer_solution.objective_value = compute_integer_objective(result.integer_solution, routes, instance);
    cout << "Objective value of the repaired solution : " << setprecision(15) << result.integer_solution.objective_value << endl;

    int elapsed_time = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - procedure_start).count();
    // Print the time it took to solve the master problem
    cout << "-----------------------------------" << endl;
    cout << "Total time spent solving the master problem : " << master_time << " ms" << endl;
    cout << "Total time spent solving the pricing problems : " << pricing_time << " ms - Average : " << pricing_time / result.number_of_iterations << " ms" << endl;
    cout << "Total time spent solving the integer problem : " << integer_time << " ms" << endl;
    cout << "Total elapsed time : " << elapsed_time << " ms" << endl;

    full_analysis(result.integer_solution, routes, instance);

    cout << "-----------------------------------" << endl;

    return result;
}