#include "column_generation.h"

#include "master_problem/rmp_solver.h"
#include "master_problem/integer_solution.h"
#include "master_problem/node.h"

#include "routes/route_optimizer.h"

#include "pricing_problem/pricing.h"

#include "data_analysis/analysis.h"

#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <execution>
#include <random>

// Removes all but the N last used routes from the routes and last_used vectors
// Returns a map from old index to new index, the new routes, and the new last_used vector
std::tuple<
    std::map<int, int>, 
    std::vector<Route>, 
    std::vector<int>> keep_last_used_routes(int N, std::vector<Route>& routes, std::vector<int>& last_used){

    std::vector<int> indices(routes.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [&last_used](int i, int j){
        return last_used[i] > last_used[j];
    });
    N = std::min(N, (int)routes.size());
    std::vector<Route> new_routes(N);
    std::vector<int> new_last_used(N);
    std::map<int, int> old_to_new;
    for (int i = 0; i < N; i++){
        new_routes[i] = routes[indices[i]];
        new_last_used[i] = last_used[indices[i]];
        old_to_new[indices[i]] = i;
    }
    return std::make_tuple(old_to_new, new_routes, new_last_used);
}

// Returns list of the vehicles to expore given the previous best reduced costs
// We base our list on a probability distrbution of the form:
// p(v) = floor + (best_v - best) / (worst - best) * (1 - floor)
// We can further adapt the floor to be higher when reduced costs are bad (with a floor of 1, we explore all vehicles)
std::vector<int> get_vehicles_to_explore(const std::vector<double> previous_best_reduced_costs) {
    using std::vector;
    // Floor is defined such that if the best reduced cost is > -100, we explore all vehicles
    // And if the best reduced cost is < -20000, we get a floor of 0.5
    double best = *std::min_element(previous_best_reduced_costs.begin(), previous_best_reduced_costs.end());
    double worst = *std::max_element(previous_best_reduced_costs.begin(), previous_best_reduced_costs.end());
    double floor = std::max(1 - 0.5 * (best + 100) / 19900, 0.5);


    vector<int> explore_list;
    for (int v = 0; v < previous_best_reduced_costs.size(); v++){
        double reduced_cost = previous_best_reduced_costs[v];
        double proba = floor + (reduced_cost - best) / (worst - best) * (1 - floor);
        double random = (double)rand() / RAND_MAX;
        if (random < proba){
            explore_list.push_back(v);
        }
    }

    return explore_list;
}



CGResult column_generation(
    const Instance & instance,
    BPNode & node,
    std::vector<Route> & routes,
    int max_resources_dominance,
    bool switch_to_cyclic_pricing,
    bool compute_integer_solution,
    int time_limit, // (in seconds)
    double reduced_cost_threshold,
    bool verbose
    ){
    using std::cout, std::endl;
    using std::setprecision, std::fixed;
    using std::vector, std::string, std::to_string;
    using std::unique_ptr;
    namespace chrono = std::chrono;

    auto start_sub_building = chrono::steady_clock::now();
    auto end_sub_building = chrono::steady_clock::now();
    int building_time = chrono::duration_cast<chrono::milliseconds>(end_sub_building - start_sub_building).count();

    int master_time = 0;
    int pricing_time = 0;

    int time_limit_ms = time_limit * 1000;

    // If the given max_resources_dominance is -1, we use all the resources
    if (max_resources_dominance == -1){
        max_resources_dominance = instance.capacities_labels.size() + 1;
    }

    // Main loop of the column generation algorithm
    int iteration = 0;
    bool stop = false;
    DualSolution previous_dual_solution = DualSolution{};
    MasterSolution solution;
    // We stop if we don't improve the objective value
    double previous_solution_objective = -1000; 
    // Testing out the sequential pricing problem solving
    int current_vehicle_index = 0;
    // We begin with the acyclic pricing which is so much faster
    // We switch to the cyclic pricing when we don't add any routes
    bool using_cyclic_pricing = false;
    // We switch to the cyclic pricing when we don't add any routes
    int n_ressources_dominance = instance.capacities_labels.size() + 1;

    // Keep the last best reduced cost for each vehicle
    vector<double> best_reduced_costs(instance.vehicles.size(), 0);

    while (!stop && master_time + pricing_time < time_limit_ms){
        // Solve the master problem
        auto start = chrono::steady_clock::now();
        solution = relaxed_RMP(instance, routes, node);
        auto end = chrono::steady_clock::now();
        int diff = chrono::duration_cast<chrono::milliseconds>(end - start).count();
        // At this point, if the solution is not feasible, it it because the cuts give a non feasible problem
        // We can stop the algorithm
        if (!solution.is_feasible){
            return CGResult{};
        }
        // Extract the dual solution from the master solution
        DualSolution& dual_solution = solution.dual_solution;

        if (verbose) {
            cout << "Iteration " << iteration << " - Objective value : " << solution.objective_value;
            cout << " - Master problem solved in " << diff << " ms \n";
            cout << "Number of interventions covered : " << setprecision(2) << count_covered_interventions(solution, routes, instance);
            std::pair<double, int> used_vehicles = count_used_vehicles(solution, routes, instance);
            cout << " - Number of vehicles used : " << used_vehicles.first << " - Unique vehicles used : " << used_vehicles.second << "\n";
        }
        master_time += diff;

        // Solve each pricing sub problem
        auto start_pricing = chrono::steady_clock::now();
        int n_added_routes = 0;
        int n_routes_changed = 0;

        // Compute with a convex combination of the previous dual solution and the current one
        double alpha = 0.5;
        DualSolution convex_dual_solution = dual_solution;
        // if (iteration > 0){
        //     convex_dual_solution = alpha * dual_solution + (1 - alpha) * previous_dual_solution;
        // }

        // Initialize the vehicle order
        vector<int> vehicle_order(instance.vehicles.size());
        std::iota(vehicle_order.begin(), vehicle_order.end(), 0);

        // Reset the best reduced costs
        double min_reduced_cost = 0;
        for (int i = 0; i < instance.vehicles.size(); i++){
            best_reduced_costs[i] = 0;
        }
        int delta = 10;
        std::vector<Route> new_routes = solve_pricing_problems_basic_pulse(
            convex_dual_solution,
            instance,
            vehicle_order,
            delta
        );
        // We add the new routes to the global routes vector
        for (Route& new_route : new_routes){
            if (new_route.reduced_cost < -reduced_cost_threshold) {
                routes.push_back(new_route);
                best_reduced_costs[new_route.vehicle_id] = std::min(best_reduced_costs[new_route.vehicle_id], new_route.reduced_cost);
                min_reduced_cost = std::min(min_reduced_cost, new_route.reduced_cost);
                node.active_routes.insert(routes.size() - 1);
                n_added_routes++;
            }
        }        

        auto end_pricing = chrono::steady_clock::now();
        int diff_pricing = chrono::duration_cast<chrono::milliseconds>(end_pricing - start_pricing).count();
        pricing_time += diff_pricing;
        if (verbose) {
            cout << "Pricing sub problems solved in " << diff_pricing << " ms - Added " << n_added_routes << " routes";
            cout << " - Min reduced cost : " << setprecision(15) << min_reduced_cost << "\n";
        }
        // ----------------- Stop conditions -----------------
        // If we added no routes but are not using the cyclic pricing yet, we switch to it
        if (n_added_routes == 0 && switch_to_cyclic_pricing && !using_cyclic_pricing){
            using_cyclic_pricing = true;
            // Reset the number of resources used for the dominance test
            n_ressources_dominance = 0;
            if (verbose){
                cout << "-----------------------------------" << endl;
                cout << "Switching to cyclic pricing" << endl;
                cout << "Current time : " << master_time + pricing_time << " ms" << endl;
            }
            // Go to the next iteration (skip the stop condition checks)
            continue;
        }
        // If we reached the end, and we were not using all the resources for the dominance test
        // We increase the number of resources used
        if (n_added_routes == 0 && using_cyclic_pricing && n_ressources_dominance < max_resources_dominance){
            n_ressources_dominance++;
            if (verbose){
                cout << "-----------------------------------" << endl;
                cout << "Increasing the number of resources used for the dominance test";
                cout << " - Now using " << n_ressources_dominance << " resources" << endl;
                cout << "Current time : " << master_time + pricing_time << " ms" << endl;
            }
            // Go to the next iteration (skip the stop condition checks)
            continue;
        }
        // If no route was added, we stop the algorithm
        if (n_added_routes == 0){
            stop = true;
        }
        previous_solution_objective = solution.objective_value;
        previous_dual_solution = dual_solution;
        iteration++;
    }

    cout << "-----------------------------------" << endl;
    if (stop) {
        cout << "Found no new route to add" << endl;
    }
    if (master_time + pricing_time >= time_limit_ms){
        cout << "Time limit reached" << endl;
    }
    cout << "End of the column generation after " << iteration << " iterations" << endl;
    cout << "Relaxed RMP objective value : " << setprecision(3) << solution.objective_value << endl;

    // Convert the value from the minimum formulation to the maximum formulation
    double total_outsource_cost = 0;
    for (int i = 0; i < instance.number_interventions; i++){
        total_outsource_cost += instance.nodes[i].duration * instance.M;
    }
    double relaxed_maximum_objective = total_outsource_cost - solution.objective_value;
    cout << "Relaxed RMP maximum formulation objective value : " << setprecision(3) << relaxed_maximum_objective << endl;

    // Update the node's upper bound
    node.upper_bound = solution.objective_value;
    // If the upper bound is lower than the lower bound, it isn't worth computing the integer solution
    if (node.upper_bound < node.lower_bound){
        compute_integer_solution = false;
    }

    IntegerSolution integer_solution = IntegerSolution{};
    int integer_time = 0;
    if (compute_integer_solution) {
        auto start_integer = chrono::steady_clock::now();
        integer_solution = integer_RMP(instance, routes, node, time_limit, false);
        auto end_integer = chrono::steady_clock::now();
        integer_time = chrono::duration_cast<chrono::milliseconds>(end_integer - start_integer).count();
        cout << "Integer RMP objective value : " << integer_solution.objective_value << endl;
        // Compute the maximum formulation objective value
        double integer_maximum_objective = total_outsource_cost - integer_solution.objective_value;
        cout << "Integer RMP maximum formulation objective value : " << setprecision(3) << integer_maximum_objective << endl;
        double gap = std::abs(solution.objective_value - integer_solution.objective_value) / integer_solution.objective_value;
        cout << "Gap between the relaxed and integer RMP : " << setprecision(5) << gap << endl;

        double gap_max = std::abs(relaxed_maximum_objective - integer_maximum_objective) / integer_maximum_objective;
        cout << "Gap between the relaxed and integer RMP (maximum formulation) : " << setprecision(5) << gap_max << endl;
    }



    // Build the result object
    CGResult result = CGResult{
        solution,
        integer_solution,
        routes,
        iteration,
        master_time,
        pricing_time,
        integer_time,
        building_time
    };

    return result;
}
