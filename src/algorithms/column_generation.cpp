#include "column_generation.h"

#include "master_problem/solver_min.h"
#include "master_problem/rmp_solver.h"
#include "master_problem/integer_solution.h"
#include "master_problem/node.h"

#include "routes/route_optimizer.h"

#include "pricing_problem/full_pricing.h"

#include "clustering/clustering.h"

#include "data_analysis/analysis.h"

#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <execution>
#include <random>

inline constexpr int S_TO_MS = 1000;


CGResult column_generation(
    const Instance & instance,
    BPNode & node,
    std::vector<Route> & routes,
    const ColumnGenerationParameters& parameters
    ){
    using std::cout, std::endl;
    using std::setprecision, std::fixed;
    using std::vector, std::string, std::to_string;
    using std::unique_ptr;
    namespace chrono = std::chrono;

    auto start_time = chrono::steady_clock::now();

    int master_time = 0;
    int pricing_time = 0;
    
    // Create the underlying RMP model
    vector<GRBVar> route_vars;
    vector<GRBVar> postpone_vars;
    vector<GRBConstr> intervention_ctrs;
    vector<GRBConstr> vehicle_ctrs;

    GRBModel model = create_model(instance, routes, route_vars, postpone_vars, intervention_ctrs, vehicle_ctrs);

    // Main loop of the column generation algorithm
    int iteration = 0;
    // Stopping conditions
    bool stop = false;
    int consecutive_non_improvement = 0;
    double previous_solution_objective = std::numeric_limits<double>::infinity();

    // Master Solutions - Do a first solve before the loop
    int status = solve_model(model);
    MasterSolution solution = extract_solution(model, route_vars, intervention_ctrs, vehicle_ctrs);
    DualSolution& dual_solution = solution.dual_solution;
    DualSolution previous_dual_solution{};

    // Objective values tracking
    vector<double> objective_values = {solution.objective_value};
    vector<int> objective_time_points = {0};

    // Order of exploration of the vehicles for the pricing problem (does not matter, we simply remove the vehicles that can not be used)
    vector<int> vehicle_order = {};
    for (int v = 0; v < instance.number_vehicles; v++){
        if (instance.vehicles[v].interventions.size() > 0){
            vehicle_order.push_back(v);
        }
    }

    // Pathwyse heuristic parameters
    bool using_cyclic_pricing = false;
    int n_ressources_dominance = instance.capacities_labels.size() + 1;
    int max_resources_dominance = parameters.max_resources_dominance;
    if (max_resources_dominance == ALL_RESOURCES_DOMINANCE){
        max_resources_dominance = instance.capacities_labels.size() + 1;
    }

    while (
        !stop && 
        !(consecutive_non_improvement == parameters.max_consecutive_non_improvement) && 
        master_time + pricing_time < S_TO_MS * parameters.time_limit){

        // ----------------- Pricing sub problem -----------------
        // Solve each pricing sub problem
        auto start_pricing = chrono::steady_clock::now();
        int n_added_routes = 0;
        int n_routes_changed = 0;

        // Compute with a convex combination of the previous dual solution and the current one
        DualSolution convex_dual_solution = dual_solution;
        if (parameters.use_stabilisation && iteration > 0){
            convex_dual_solution = parameters.alpha * dual_solution + (1 - parameters.alpha) * previous_dual_solution;
        }

        // Use the pricing algorithm defined in the parameters
        vector<Route> new_routes;
        if (parameters.pricing_function == PRICING_PATHWYSE_BASIC){
            new_routes = full_pricing_problems_basic(
                convex_dual_solution,
                instance,
                vehicle_order,
                using_cyclic_pricing,
                n_ressources_dominance
            );
        } else if (parameters.pricing_function == PRICING_DIVERSIFICATION){
            new_routes = full_pricing_problems_diversification(
                convex_dual_solution,
                instance,
                vehicle_order,
                using_cyclic_pricing,
                n_ressources_dominance,
                iteration
            );
        } else if (parameters.pricing_function == PRICING_CLUSTERING){
            new_routes = full_pricing_problems_clustering(
                convex_dual_solution,
                instance,
                vehicle_order,
                using_cyclic_pricing,
                n_ressources_dominance,
                iteration
            );
        } else if (parameters.pricing_function == PRICING_PA_BASIC){
            new_routes = full_pricing_problems_basic_pulse(
                convex_dual_solution,
                instance,
                vehicle_order,
                parameters.delta,
                parameters.solution_pool_size,
                parameters.pricing_verbose
            );
        } else if (parameters.pricing_function == PRICING_PA_GROUPED){
            auto vehicle_groups = regroup_vehicles_by_depot(instance.vehicles);
            new_routes = full_pricing_problems_grouped_pulse(
                convex_dual_solution,
                instance,
                vehicle_groups,
                parameters.delta,
                parameters.solution_pool_size,
                parameters.pricing_verbose
            );
        } else if (parameters.pricing_function == PRICING_MPA) {
            new_routes = full_pricing_problems_multithreaded_pulse(
                convex_dual_solution,
                instance,
                vehicle_order,
                parameters.delta,
                parameters.solution_pool_size,
                parameters.pricing_verbose
            );
        } else if (parameters.pricing_function == PRICING_MPA_GROUPED){
            auto vehicle_groups = regroup_vehicles_by_depot(instance.vehicles);
            new_routes = full_pricing_problems_grouped_pulse_multithreaded(
                convex_dual_solution,
                instance,
                vehicle_groups,
                parameters.delta,
                parameters.solution_pool_size,
                parameters.pricing_verbose
            );
        } else if (parameters.pricing_function == PRICING_MPA_GROUPED_PAR){
            auto vehicle_groups = regroup_vehicles_by_depot(instance.vehicles);
            new_routes = full_pricing_problems_grouped_pulse_par_par(
                convex_dual_solution,
                instance,
                vehicle_groups,
                parameters.delta,
                parameters.solution_pool_size,
                parameters.pricing_verbose
            );
        } else if (parameters.pricing_function == PRICING_PW_PA) {
            // Begin by solving the Pathwyse heuristic
            if (!using_cyclic_pricing){
                new_routes = full_pricing_problems_basic(
                    convex_dual_solution,
                    instance,
                    vehicle_order,
                    using_cyclic_pricing,
                    n_ressources_dominance
                );
            } else {
                // Then, solve the PA
                new_routes = full_pricing_problems_basic_pulse(
                    convex_dual_solution,
                    instance,
                    vehicle_order,
                    parameters.delta,
                    parameters.solution_pool_size,
                    parameters.pricing_verbose
                );
            }
        }

        // We add the new routes to the global routes vector
        double min_reduced_cost = std::numeric_limits<double>::infinity();
        for (Route& new_route : new_routes){
            min_reduced_cost = std::min(min_reduced_cost, new_route.reduced_cost);
            if (new_route.reduced_cost < - parameters.reduced_cost_threshold) {
                routes.push_back(new_route);
                node.active_routes.insert(routes.size() - 1);
                n_added_routes++;

                // Add the route to the model
                add_route(model, new_route, instance, route_vars, intervention_ctrs, vehicle_ctrs);
            }
        }        
        auto end_pricing = chrono::steady_clock::now();
        int diff_pricing = chrono::duration_cast<chrono::milliseconds>(end_pricing - start_pricing).count();
        pricing_time += diff_pricing;
        
        if (parameters.verbose) {
            cout << "Pricing sub problems solved in " << diff_pricing << " ms - Added " << n_added_routes << " routes";
            cout << " - Min reduced cost : " << setprecision(15) << min_reduced_cost << "\n";
        }

        // ----------------- Master problem -----------------
        // Solve the master problem
        auto start = chrono::steady_clock::now();
        int status = solve_model(model);
        solution = extract_solution(model, route_vars, intervention_ctrs, vehicle_ctrs);
        auto end = chrono::steady_clock::now();
        int diff = chrono::duration_cast<chrono::milliseconds>(end - start).count();
        // At this point, if the solution is not feasible, it it because the cuts give a non feasible problem
        // We can stop the algorithm
        if (!solution.is_feasible){
            return CGResult{};
        }
        // Extract the dual solution from the master solution
        DualSolution& dual_solution = solution.dual_solution;

        // Update the objective tracking
        objective_values.push_back(solution.objective_value);
        objective_time_points.push_back(chrono::duration_cast<chrono::milliseconds>(end - start_time).count());

        if (parameters.verbose) {
            cout << "Iteration " << iteration << " - Objective value : " << solution.objective_value;
            cout << " - Master problem solved in " << diff << " ms \n";
            cout << "Number of interventions covered : " << setprecision(3) << count_covered_interventions(solution, routes, instance);
            std::pair<double, int> used_vehicles = count_used_vehicles(solution, routes, instance);
            cout << " - Number of vehicles used : " << used_vehicles.first << " - Unique vehicles used : " << used_vehicles.second << "\n";
        }
        master_time += diff;

        // ----------------- Stop conditions -----------------
        // If we added no routes but are not using the cyclic pricing yet, we switch to it
        // Only for non PA algorithms
        if (std::find(PA_VARIATIONS.begin(), PA_VARIATIONS.end(), parameters.pricing_function) == PA_VARIATIONS.end()){
            if (n_added_routes == 0 && parameters.switch_to_cyclic_pricing && !using_cyclic_pricing){
                using_cyclic_pricing = true;
                // Reset the number of resources used for the dominance test
                n_ressources_dominance = 0;
                if (parameters.verbose){
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
                if (parameters.verbose){
                    cout << "-----------------------------------" << endl;
                    cout << "Increasing the number of resources used for the dominance test";
                    cout << " - Now using " << n_ressources_dominance << " resources" << endl;
                    cout << "Current time : " << master_time + pricing_time << " ms" << endl;
                }
                // Go to the next iteration (skip the stop condition checks)
                continue;
            }
        }
        // If no route was added, we stop the algorithm
        if (n_added_routes == 0){
            stop = true;
        }
        // Count the number of consecutive non improvement
        if (solution.objective_value >= previous_solution_objective){
            consecutive_non_improvement++;
        } else {
            consecutive_non_improvement = 0;
        }
        previous_solution_objective = solution.objective_value;
        previous_dual_solution = dual_solution;
        iteration++;
    }

    cout << "-----------------------------------" << endl;
    if (stop) {
        cout << "Found no new route to add" << endl;
    }
    if (consecutive_non_improvement == parameters.max_consecutive_non_improvement){
        cout << "Stopped after " << parameters.max_consecutive_non_improvement << " iterations without improvement" << endl;
    }
    if (master_time + pricing_time >= S_TO_MS * parameters.time_limit){
        cout << "Time limit reached" << endl;
    }
    cout << "End of the column generation after " << iteration << " iterations" << endl;
    cout << "Relaxed RMP objective value : " << setprecision(15) << solution.objective_value << endl;

    // Convert the value from the minimum formulation to the maximum formulation
    double total_outsource_cost = 0;
    for (int i = 0; i < instance.number_interventions; i++){
        total_outsource_cost += instance.nodes[i].duration * instance.M;
    }
    double relaxed_maximum_objective = total_outsource_cost - solution.objective_value;
    cout << "Relaxed RMP maximum formulation objective value : " << setprecision(15) << relaxed_maximum_objective << endl;

    // Update the node's upper bound
    node.upper_bound = solution.objective_value;
    // If the upper bound is lower than the lower bound, it isn't worth computing the integer solution
    bool compute_integer_solution = parameters.compute_integer_solution;
    if (node.upper_bound < node.lower_bound){
        compute_integer_solution = false;
    }

    IntegerSolution integer_solution = IntegerSolution{};
    int integer_time = 0;
    if (compute_integer_solution) {
        auto start_integer = chrono::steady_clock::now();
        set_integer_variables(model, route_vars, postpone_vars);
        int status = solve_model(model);
        integer_solution = extract_integer_solution(model, route_vars);
        auto end_integer = chrono::steady_clock::now();
        integer_time = chrono::duration_cast<chrono::milliseconds>(end_integer - start_integer).count();
        cout << "Integer RMP objective value : " << integer_solution.objective_value << endl;
        // Compute the maximum formulation objective value
        double integer_maximum_objective = total_outsource_cost - integer_solution.objective_value;
        cout << "Integer RMP maximum formulation objective value : " << setprecision(15) << integer_maximum_objective << endl;
        double gap = std::abs(solution.objective_value - integer_solution.objective_value) / integer_solution.objective_value;
        cout << "Gap between the relaxed and integer RMP : " << setprecision(5) << gap << endl;
        double gap_max = std::abs(relaxed_maximum_objective - integer_maximum_objective) / integer_maximum_objective;
        cout << "Gap between the relaxed and integer RMP (maximum formulation) : " << setprecision(5) << gap_max << endl;
    }



    // Build the result object
    CGResult result = CGResult{
        solution,
        integer_solution,
        iteration,
        master_time,
        pricing_time,
        integer_time,
        objective_values,
        objective_time_points
    };

    return result;
}
