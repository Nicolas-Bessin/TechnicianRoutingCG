#include "column_generation.h"

#include "master_problem/rmp_solver.h"
#include "master_problem/node.h"

#include "routes/route_optimizer.h"

#include "pricing_problem/pricing.h"

#include "data_analysis/analysis.h"

#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <execution>




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
    MasterSolution solution;
    // We stop if we don't improve the objective value
    double previous_solution_objective = -1000; 
    // Testing out the sequential pricing problem solving
    int current_vehicle_index = 0;
    // We begin with the acyclic pricing which is so much faster
    // We switch to the cyclic pricing when we don't add any routes
    bool using_cyclic_pricing = false;
    // Initially, while using the acyclic pricing, we can use all the resources for the dominance test
    int n_ressources_dominance = 0;

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
        if (verbose) {
            cout << "Iteration " << iteration << " - Objective value : " << solution.objective_value << "\n";
            cout << "Master problem solved in " << diff << " ms \n";
            cout << "Number of interventions covered : " << setprecision(2) << count_covered_interventions(solution, routes, instance);
            std::pair<double, int> used_vehicles = count_used_vehicles(solution, routes, instance);
            cout << " - Number of vehicles used : " << used_vehicles.first << " - Unique vehicles used : " << used_vehicles.second << "\n";
            cout << "Min alpha : " << *std::min_element(solution.alphas.begin(), solution.alphas.end());
            cout << " - Max alpha : " << *std::max_element(solution.alphas.begin(), solution.alphas.end());
            cout << " - Min beta : " << *std::min_element(solution.betas.begin(), solution.betas.end());
            cout << " - Max beta : " << *std::max_element(solution.betas.begin(), solution.betas.end()) << "\n";
        }
        master_time += diff;

        // Solve each pricing sub problem
        auto start_pricing = chrono::steady_clock::now();
        int n_added_routes = 0;
        int n_routes_changed = 0;

        double max_reduced_cost = 0;

        std::vector<Route> new_routes = solve_pricing_problems_basic(
            solution,
            instance,
            using_cyclic_pricing,
            n_ressources_dominance,
            reduced_cost_threshold
        );
        n_added_routes = new_routes.size();
        // We add the new routes to the global routes vector
        for (Route& new_route : new_routes){
            routes.push_back(new_route);
            node.active_routes.insert(routes.size() - 1);
        }
        

        auto end_pricing = chrono::steady_clock::now();
        int diff_pricing = chrono::duration_cast<chrono::milliseconds>(end_pricing - start_pricing).count();
        pricing_time += diff_pricing;
        if (verbose) {
            cout << "Pricing sub problems solved in " << diff_pricing << " ms - Added " << n_added_routes << " routes";
            cout << " - Optimized " << n_routes_changed << " routes\n";
            cout << " - Max reduced cost : " << setprecision(15) << max_reduced_cost << "\n";
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

    // Update the node's upper bound
    node.upper_bound = solution.objective_value;
    // If the upper bound is lower than the lower bound, it isn't worth computing the integer solution
    if (node.upper_bound < node.lower_bound){
        compute_integer_solution = false;
    }

    IntegerSolution integer_solution = IntegerSolution{};
    int integer_time = 0;
    if (compute_integer_solution) {
        // Solve the integer version of the problem
        auto start_integer = chrono::steady_clock::now();
        integer_solution = integer_RMP(instance, routes, node, false);
        auto end_integer = chrono::steady_clock::now();
        integer_time = chrono::duration_cast<chrono::milliseconds>(end_integer - start_integer).count();
        cout << "Integer RMP objective value : " << integer_solution.objective_value << endl;
        double gap = (solution.objective_value - integer_solution.objective_value) / integer_solution.objective_value;
        cout << "Gap between the relaxed and integer RMP : " << setprecision(3) << gap << endl;
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
