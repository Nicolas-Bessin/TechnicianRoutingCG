#include "src/parser.h"
#include "src/preprocessing.h"

#include "src/master.h"
#include "src/RMP_solver.h"
#include "src/pricing.h"
#include "src/route_optimizer.h"

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

#define TIME_LIMIT 1200
#define SOLVER_MODE IMPOSE_ROUTING
#define THRESHOLD 1e-6
#define VERBOSE true
#define GREEDY_INIT false
#define CYCLIC_PRICING true
#define MAX_ITER 10000
#define COMPUTE_INTEGER_SOL true
#define N_INTERVENTIONS 77

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
    string default_filename = "../data/instance_1.json";
    Instance instance = parse_file(default_filename, true);

    // // Only keep the first 25 nodes
    // vector<int> kept_nodes = vector<int>(instance.number_interventions);
    // for (int i = 0; i < N_INTERVENTIONS; i++){
    //     kept_nodes[i] = 1;
    // }
    // instance = cut_instance(instance, kept_nodes);

    // Check wether the time and distance matrices are symetric
    cout << "Distance matrix is symetric : " << is_symmetric(instance.distance_matrix);
    cout << " - Biggest gap : " << symmetry_gap(instance.distance_matrix) << endl;
    cout << "Time matrix is symetric : " << is_symmetric(instance.time_matrix);
    cout << " - Biggest gap : " << symmetry_gap(instance.time_matrix) << endl;

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
    cout << "Starting the column generation algorithm" << endl;


    // Create a root node for the algorithm
    BPNode root = RootNode(routes);
    CGResult result = column_generation(instance, root, routes, THRESHOLD, TIME_LIMIT, MAX_ITER, CYCLIC_PRICING, COMPUTE_INTEGER_SOL, VERBOSE);

    // Extract the results from the column generation algorithm
    int master_time = result.master_time;
    int pricing_time = result.pricing_time;
    int integer_time = result.integer_time;
    int sub_building_time = result.building_time;

    MasterSolution master_solution = result.master_solution;
    IntegerSolution integer_solution = result.integer_solution;


    // If the integer solution is not feasible, we can't do much
    if (!integer_solution.is_feasible){
        cout << "-----------------------------------" << endl;
        cout << "The integer solution is not feasible" << endl;
        return 1;
    }

    // Print the routes in the integer solution (in detail)
    // full_analysis(integer_solution, routes, instance);

    int elapsed_time = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start_parse).count();
    // Print the time it took to solve the master problem
    cout << "-----------------------------------" << endl;
    cout << "Total time spent building the pricing problems : " << sub_building_time << " ms" << endl;
    cout << "Total time spent solving the master problem : " << master_time << " ms" << endl;
    cout << "Total time spent solving the pricing problems : " << pricing_time << " ms - Average : " << pricing_time / result.number_of_iterations << " ms" << endl;
    cout << "Total time spent solving the integer problem : " << integer_time << " ms" << endl;
    cout << "Total elapsed time : " << elapsed_time << " ms" << endl;

    full_analysis(integer_solution, routes, instance);
    cout << "True cost of the integer solution : " << compute_integer_objective(integer_solution, routes, instance) << endl;

    cout << "-----------------------------------" << endl;
    //print_used_routes(integer_solution, routes, instance);

    IntegerSolution optimized_solution = optimize_routes(integer_solution, routes, instance);
    cout << "Objective value of the optimized solution : " << optimized_solution.objective_value << endl;

    

    return 0;
}

    // vector<Route> best_routes = parse_routes_from_file("../routes/best_small.json", instance);
    // // Compute the reduced cost of the best routes with respect to the master solution
    // for (const Route &route : best_routes){
    //     double reduced_cost = compute_reduced_cost(route, master_solution.alphas, master_solution.betas[route.vehicle_id], instance);
    //     cout << "Reduced cost of the best route " << route.vehicle_id << " : " << reduced_cost << endl;
    // }
    // cout << "-----------------------------------" << endl;
    // // Convert those routes to a set a required edges
    // set<tuple<int, int, int>> required_edges = routes_to_required_edges(best_routes);
    // set<tuple<int, int, int>> forbidden_edges = set<tuple<int, int, int>>();
    // vector<int> order = {14, 3, 8, 7, 19, 6, 10, 0, 1, 2};

    // vector<Route> regenerated_routes = greedy_heuristic_duals(instance, master_solution, CYCLIC_PRICING, order, forbidden_edges, required_edges);
    // BPNode regenerated_root = RootNode(regenerated_routes);
    // IntegerSolution regenerated_solution = integer_RMP(instance, regenerated_routes, regenerated_root);

    // // Print the reduced costs of the regenerated routes
    // for (const Route &route : regenerated_routes){
    //     double reduced_cost = compute_reduced_cost(route, master_solution.alphas, master_solution.betas[route.vehicle_id], instance);
    //     cout << "Route v" << route.vehicle_id << " - computed RC : " << reduced_cost;
    //     cout << " - returned RC : " << route.reduced_cost << endl;
    // }

    // cout << "-----------------------------------" << endl;
    // cout << "Generating a new route for vehicle 14 given the final dual - without using the known best routes" << endl;
    // // Generate a route for vehicle 14
    // unique_ptr<Problem> pricing_problem_normal = create_pricing_instance(instance, instance.vehicles[14], CYCLIC_PRICING);
    // update_pricing_instance(pricing_problem_normal, master_solution, instance, instance.vehicles[14]);
    // Route new_route_14_normal = solve_pricing_problem(pricing_problem_normal, instance, instance.vehicles[14]);
    // // Print its reduced cost
    // double reduced_cost = compute_reduced_cost(new_route_14_normal, master_solution.alphas, master_solution.betas[14], instance);
    // cout << "Reduced cost for vehicle 14: " << new_route_14_normal.reduced_cost << endl;
    // cout << "Reduced cost for vehicle 14 (computed): " << reduced_cost << endl;

    // // Print the route itself
    // print_route(new_route_14_normal, instance, master_solution);

    // cout << "-----------------------------------" << endl;
    // int N_EDGES = 1;
    // cout << "Generating a new route for vehicle 14 given the final dual - imposing the first " << N_EDGES << " edges of best route for v14" <<  endl;
    // // Create a set of edges containing the first N_EDGES of the best route for vehicle 14
    // set<tuple<int, int, int>> edges_14;
    // for (int i = 0; i < N_EDGES; i++){
    //     int edge_i = regenerated_routes[0].id_sequence.at(i);
    //     int edge_j = regenerated_routes[0].id_sequence.at(i + 1);
    //     edges_14.insert(std::make_tuple(edge_i, edge_j, 14));
    // }
    // set<tuple<int, int, int>> empty_edges = set<tuple<int, int, int>>();
    // // Generate a route for vehicle 14
    // unique_ptr<Problem> pricing_problem_impose = create_pricing_instance(instance, instance.vehicles[14], CYCLIC_PRICING, empty_edges, edges_14);
    // update_pricing_instance(pricing_problem_impose, master_solution, instance, instance.vehicles[14]);
    // Route new_route_14_impose = solve_pricing_problem(pricing_problem_impose, instance, instance.vehicles[14]);
    // // Print its reduced cost
    // double reduced_cost_impose = compute_reduced_cost(new_route_14_impose, master_solution.alphas, master_solution.betas[14], instance);
    // cout << "Reduced cost for vehicle 14: " << new_route_14_impose.reduced_cost << endl;
    // cout << "Reduced cost for vehicle 14 (computed): " << reduced_cost_impose << endl;

    // // Print the route itself
    // print_route(new_route_14_impose, instance, master_solution);

    // cout << "-----------------------------------" << endl;
    // cout << "Generating a new route for vehicle 14 given the final dual - forbidding the first " << N_EDGES << " edges of bad route for v14" <<  endl;
    // // Create a set of edges containing the first N_EDGES of the bad route for vehicle 14
    // set<tuple<int, int, int>> edges_14_forbid;
    // for (int i = 0; i < N_EDGES; i++){
    //     int edge_i = new_route_14_normal.id_sequence.at(i);
    //     int edge_j = new_route_14_normal.id_sequence.at(i + 1);
    //     edges_14_forbid.insert(std::make_tuple(edge_i, edge_j, 14));
    // }
    // // Generate a route for vehicle 14
    // unique_ptr<Problem> pricing_problem_forbid = create_pricing_instance(instance, instance.vehicles[14], CYCLIC_PRICING, edges_14_forbid, empty_edges);
    // update_pricing_instance(pricing_problem_forbid, master_solution, instance, instance.vehicles[14]);
    // Route new_route_14_forbid = solve_pricing_problem(pricing_problem_forbid, instance, instance.vehicles[14]);
    // // Print its reduced cost
    // double reduced_cost_forbid= compute_reduced_cost(new_route_14_forbid, master_solution.alphas, master_solution.betas[14], instance);
    // cout << "Reduced cost for vehicle 14: " << new_route_14_forbid.reduced_cost << endl;
    // cout << "Reduced cost for vehicle 14 (computed): " << reduced_cost_forbid<< endl;

    // // Print the route itself
    // print_route(new_route_14_forbid, instance, master_solution);
