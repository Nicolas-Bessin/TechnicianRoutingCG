#include "src/parser.h"
#include "src/preprocessing.h"

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
#define VERBOSE true
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

    // Check wether the time and distance matrices are symetric
    cout << "Distance matrix is symetric : " << is_symmetric(instance.distance_matrix) << " - Biggest gap : " << symmetry_gap(instance.distance_matrix) << endl;
    cout << "Time matrix is symetric : " << is_symmetric(instance.time_matrix) << " - Biggest gap : " << symmetry_gap(instance.time_matrix) << endl;

    preprocess_interventions(instance);

    auto end_parse = chrono::steady_clock::now();
    int diff_parse = chrono::duration_cast<chrono::milliseconds>(end_parse - start_parse).count();

    cout << "Total time spent parsing the instance : " << diff_parse << " ms" << endl;

    vector<Route> routes = parse_routes_from_file("../routes/best_small.json", instance);

    cout << "-----------------------------------" << endl;
    // Get an integer solution from the routes
    BPNode root = RootNode(routes);
    IntegerSolution integer_solution = integer_RMP(instance, routes, root);

    print_used_routes(integer_solution, routes, instance);

    full_analysis(integer_solution, routes, instance);

    cout << "True cost of the integer solution : " << setprecision(10) <<  compute_integer_objective(integer_solution, routes, instance) << endl;

    cout << "-----------------------------------" << endl;
    // Convert those routes to a set a required edges
    set<tuple<int, int, int>> required_edges = routes_to_required_edges(routes);
    set<tuple<int, int, int>> forbidden_edges = set<tuple<int, int, int>>();
    vector<int> order = {14, 3, 8, 7, 19, 6, 10, 0, 1, 2};

    vector<Route> regenerated_routes = greedy_heuristic(instance, order, forbidden_edges, required_edges);
    BPNode regenerated_root = RootNode(regenerated_routes);
    IntegerSolution regenerated_solution = integer_RMP(instance, regenerated_routes, regenerated_root);

    print_used_routes(regenerated_solution, regenerated_routes, instance);
    
    full_analysis(regenerated_solution, regenerated_routes, instance);

    // cout << "-----------------------------------" << endl;
    // vector<Route> routes;
    // if (GREEDY_INIT) {
    //     cout << "Initializing the routes with a greedy heuristic" << endl;
    //     routes = greedy_heuristic(instance);
    //     IntegerSolution greedy_solution = IntegerSolution(vector<int>(routes.size(), 1), 0);
    //     greedy_solution.objective_value = compute_integer_objective(greedy_solution, routes, instance);
    //     cout << "Objective value of the greedy heuristic : " << greedy_solution.objective_value << endl;
    // } else {
    //     cout << "Initializing the routes with an empty route" << endl;
    //     routes = vector<Route>();
    //     routes.push_back(Route(instance.nodes.size()));
    // }

    // cout << "-----------------------------------" << endl;
    // cout << "Starting the column generation algorithm" << endl;

    // // Global time limit for the column generation algorithm of 60 seconds
    // const int time_limit = TIME_LIMIT * 1000;

    // // Create a root node for the algorithm
    // BPNode root = RootNode(routes);
    // CGResult result = column_generation(instance, root, routes, THRESHOLD, time_limit, 10000, true, VERBOSE);

    // // Extract the results from the column generation algorithm
    // int master_time = result.master_time;
    // int pricing_time = result.pricing_time;
    // int integer_time = result.integer_time;
    // int sub_building_time = result.building_time;

    // MasterSolution master_solution = result.master_solution;
    // IntegerSolution integer_solution = result.integer_solution;


    // // If the integer solution is not feasible, we can't do much
    // if (!integer_solution.is_feasible){
    //     cout << "-----------------------------------" << endl;
    //     cout << "The integer solution is not feasible" << endl;
    //     return 1;
    // }

    // // Print the routes in the integer solution (in detail)
    // // full_analysis(integer_solution, routes, instance);

    // int elapsed_time = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start_parse).count();
    // // Print the time it took to solve the master problem
    // cout << "-----------------------------------" << endl;
    // cout << "Total time spent building the pricing problems : " << sub_building_time << " ms" << endl;
    // cout << "Total time spent solving the master problem : " << master_time << " ms" << endl;
    // cout << "Total time spent solving the pricing problems : " << pricing_time << " ms - Average : " << pricing_time / result.number_of_iterations << " ms" << endl;
    // cout << "Total time spent solving the integer problem : " << integer_time << " ms" << endl;
    // cout << "Total elapsed time : " << elapsed_time << " ms" << endl;

    // full_analysis(integer_solution, routes, instance);
    // cout << "True cost of the integer solution : " << compute_integer_objective(integer_solution, routes, instance) << endl;

    // cout << "-----------------------------------" << endl;
    // //print_used_routes(integer_solution, routes, instance);


    return 0;
}

    // cout << " Starting the compact solver using mode " << SOLVER_MODE << endl;

    // int remaining_time = TIME_LIMIT - elapsed_time / 1000;
    // // Keep only the routes that are used in the integer solution
    // vector<Route> used_routes = keep_used_routes(routes, integer_solution);
    // // We can then solve the compact model with these imposed routings
    // CompactSolution<int> compact_solution = compact_solver(instance, remaining_time, used_routes, SOLVER_MODE, true);
    // // Convert back to routes
    // vector<Route> compact_routes = compact_solution_to_routes(instance, compact_solution);
    // // Create a dummy integer solution (all variables set to 1)
    // IntegerSolution compact_integer_solution = IntegerSolution(vector<int>(compact_routes.size(), 1), compact_solution.objective_value);

    // cout << "Manual computing of the compact solution value : " << compute_integer_objective(compact_integer_solution, compact_routes, instance) << endl;

    // cout << "-----------------------------------" << endl;
    // // Print the routes in the compact solution
    // // print_used_routes(compact_integer_solution, compact_routes, instance);



    // // Run the analysis on the compact solution
    // // full_analysis(compact_integer_solution, compact_routes, instance);

    // // Re-compute the master objective value using the compact routes only first
    // MasterSolution compact_master_solution = relaxed_RMP(instance, compact_routes);
    // cout << "Objective value of the RMP with only the routes from the compact formulation : " << compact_master_solution.objective_value << endl;
    // // Then, add those routes to the existing routes and re-solve the master problem
    // routes.insert(routes.end(), compact_routes.begin(), compact_routes.end());
    // MasterSolution new_master_solution = relaxed_RMP(instance, routes);
    // cout << "Objective value of the RMP with the routes from the compact formulation added : " << new_master_solution.objective_value << endl;



    // // Convert our master solution to a compact solution
    // CompactSolution<double> compact_solution = to_compact_solution(master_solution, routes, instance);
    // // Evaluate the objective value of the compact solution
    // double compact_objective = evaluate_compact_solution(compact_solution, instance);
    // cout << "Objective value of the relaxed converted compact solution : " << compact_objective << endl;

    // // Do the same thing for the integer solution
    // CompactSolution<int> compact_integer_solution = to_compact_solution(integer_solution, routes, instance);
    // // Evaluate the objective value of the compact solution
    // double compact_integer_objective = evaluate_compact_solution(compact_integer_solution, instance);
    // cout << "Objective value of the integer converted compact solution : " << compact_integer_objective << endl;


    

    
    // // Generate new routes using the greedy heuristic
    // cout << "Adding new routes using the greedy heuristic" << endl;
    // vector<Route> new_routes = greedy_heuristic_alphas(instance);
    // // Compute the standalone value of the new routes
    // IntegerSolution new_routes_solution = IntegerSolution(vector<int>(new_routes.size(), 1), 0);
    // new_routes_solution.objective_value = compute_integer_objective(new_routes_solution, new_routes, instance);
    // cout << "Objective value using only the new routes : " << new_routes_solution.objective_value << endl;
    // // Check the feasibility of the routes we have generated (this call checks wether interventions are covered more than once)
    // covered_interventions(new_routes_solution, new_routes, instance);
    // // Compute the reduced costs of the new routes in the master problem
    // double min_reduced_cost = +INFINITY;
    // double max_reduced_cost = -INFINITY;
    // Route& best_route = new_routes[0];
    // for (Route &route : new_routes){
    //     double reduced_cost = compute_reduced_cost(route, master_solution.alphas, master_solution.betas[route.vehicle_id], instance);
    //     min_reduced_cost = std::min(min_reduced_cost, reduced_cost);
    //     if (reduced_cost > max_reduced_cost){
    //         max_reduced_cost = reduced_cost;
    //         best_route = route;
    //     }
    // }
    // cout << "Minimum reduced cost of the new routes : " << min_reduced_cost << endl;
    // cout << "Maximum reduced cost of the new routes : " << max_reduced_cost << endl;
    // if (max_reduced_cost > THRESHOLD){
    //     cout << "Route with the highest >0 reduced cost : " << endl;
    //     print_route(best_route, instance);
    // }
    // // Add them to the existing routes
    // routes.insert(routes.end(), new_routes.begin(), new_routes.end());
    // cout << "-----------------------------------" << endl;
    // cout << "Re-solving the master and integer problems with the new routes" << endl;
    // // Re-compute the master then the integer solution
    // MasterSolution new_master_solution = relaxed_RMP(instance, routes);
    // cout << "Objective value of the new master solution : " << new_master_solution.objective_value << endl;
    // IntegerSolution new_integer_solution = integer_RMP(instance, routes);
    // cout << "Objective value of the new integer solution : " << new_integer_solution.objective_value << endl;




    // cout << "Generating a route that can cover unassigned interventions" << endl;
    // vector<int> covered = covered_interventions(integer_solution, routes, instance);
    // int vehicle_id = 10;
    // Vehicle vehicle = vehicle_mask(instance.vehicles[vehicle_id], covered);
    // unique_ptr<Problem> problem = create_pricing_instance(instance, vehicle);
    // update_pricing_instance(problem, vector<double>(instance.number_interventions, 0), instance, vehicle);
    // Route new_route = solve_pricing_problem(problem, 1, instance, vehicle)[0];
    // // Add the new route to the existing routes
    // routes.push_back(new_route);
    // // Re-solve the master and integer problems
    // MasterSolution new_master_solution = relaxed_RMP(instance, routes);
    // IntegerSolution new_integer_solution = integer_RMP(instance, routes);
    // cout << "Objective value of the new master solution : " << new_master_solution.objective_value << endl;
    // cout << "Objective value of the new integer solution : " << new_integer_solution.objective_value << endl;

    // full_analysis(new_integer_solution, routes, instance);
    



    // cout << "One round of generation in the gap" << endl;
    // double gap = integer_solution.objective_value - master_solution.objective_value;
    // cout << "Gap : " << gap << endl;
    // CGResult gap_result = column_generation(instance, routes, gap, time_limit, 1, VERBOSE);
    // // Insert the new routes in the existing routes
    

    // full_analysis(gap_result.integer_solution, gap_result.routes, instance);