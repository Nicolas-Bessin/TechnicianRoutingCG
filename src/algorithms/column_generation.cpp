#include "column_generation.h"

#include "instance/preprocessing.h"

#include "master_problem/master_solver.h"
#include "master_problem/rmp_solver.h"
#include "master_problem/node.h"

#include "routes/route_optimizer.h"

#include "../../pathwyse/core/utils/param.h"

#include "pricing_problem/full_pricing.h"

#include "clustering/clustering.h"

#include "data_analysis/analysis.h"

#include "repair/repair.h"

#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <execution>
#include <random>
#include <thread>

inline constexpr int S_TO_MS = 1000;


std::map<std::string, std::any> pathwyse_parameters_dict(
    const ColumnGenerationParameters& parameters,
    double remaining_time,
    bool using_cyclic_pricing
){
    return std::map<std::string, std::any>({
        {"time_limit", remaining_time},
        {"verbosity", -1},
        {"use_visited", parameters.use_visited},
        {"ng", parameters.ng},
        {"dssr", parameters.dssr},
        {"compare_unreachables", using_cyclic_pricing},
        {"bidirectional", parameters.bidirectional_DP}
    });
}


CGResult column_generation(
    const Instance & instance,
    std::vector<Route> & routes,
    const ColumnGenerationParameters& parameters
    ){
    using std::cout, std::endl;
    using std::setprecision, std::fixed;
    using std::vector, std::string, std::to_string;
    using std::unique_ptr;
    namespace chrono = std::chrono;

    auto procedure_start = chrono::steady_clock::now();

    int master_time = 0;
    int pricing_time = 0;
    
    // Create the underlying RMP model
    vector<GRBVar> route_vars;
    vector<GRBVar> postpone_vars;
    vector<GRBConstr> intervention_ctrs;
    vector<GRBConstr> vehicle_ctrs;
    GRBModel model = create_model(
        instance, 
        routes, 
        route_vars, 
        postpone_vars, 
        intervention_ctrs,
        vehicle_ctrs,
        parameters.solver_objective_mode
    );

    // During the first phase of the sequential CG, we stop as soon as we reach the minimum outsourced duration given by a preprocessing of trivially non feasible interventions
    int duration_lower_bound = 0;
    if (parameters.solver_objective_mode == SolverMode::DURATION_ONLY){
        duration_lower_bound = min_a_priori_outsourced_time(instance, parameters.verbose);
    }

    // If in the second phase of the sequential CG, add the constraint of maximum duration outsourced
    if (parameters.max_outsourced_duration >= 0 && parameters.solver_objective_mode == SolverMode::SOLUTION_MINIMISATION){
        add_max_outsourced_duration_constraint(model, instance, postpone_vars, parameters.max_outsourced_duration);
    }

    // Main loop of the column generation algorithm
    int iteration = 0;
    // Stopping conditions
    bool stop = false;
    int consecutive_non_improvement = 0;
    double previous_solution_objective = std::numeric_limits<double>::infinity();

    // Master Solutions - Do a first solve before the loop
    int status = solve_model(model);
    MasterSolution solution = extract_solution(model, route_vars, intervention_ctrs, vehicle_ctrs);
    if (parameters.verbose){
        cout << "Initial solution : " << solution.objective_value << endl;
    }
    DualSolution& dual_solution = solution.dual_solution;
    DualSolution previous_dual_solution{};

    // Intermediary integer solutions
    IntegerSolution intermediary_integer_solution {};
    vector<Route> intermediary_integer_routes {};
    if (parameters.compute_intermediate_integer_solutions){
        intermediary_integer_solution = solve_intermediary_integer_model(model, route_vars, postpone_vars, parameters.time_limit);
    }

    // Objective values tracking
    vector<double> objective_values = {solution.objective_value};
    vector<double> solution_costs = {relaxed_solution_cost(solution, routes)};
    vector<double> covered_interventions = {count_covered_interventions(solution, routes, instance)};
    vector<double> time_points = {0.0};

    // Intermediate integer solution tracking
    vector<double> integer_objective_values = {};
    vector<int> integer_covered_interventions = {};
    vector<double> integer_solution_costs = {};
    if (parameters.compute_intermediate_integer_solutions){
        integer_objective_values.push_back(intermediary_integer_solution.objective_value);
        integer_covered_interventions.push_back(count_covered_interventions(intermediary_integer_solution, routes, instance));
        integer_solution_costs.push_back(integer_solution_cost(intermediary_integer_solution, routes));
    }

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

    auto start_time = chrono::steady_clock::now();

    while (!stop) {

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
        // Get the remaining time to be set as the time limit for the Pathwyse heuristic
        int remaining_time_ms = S_TO_MS * parameters.time_limit - (master_time + pricing_time);
        double remaining_time = ((remaining_time_ms / 1000.0) / instance.number_vehicles) * std::thread::hardware_concurrency();
        vector<Route> new_routes;
        if (parameters.pricing_function == PRICING_PATHWYSE_BASIC){
            Parameters::setParametersFromDict(pathwyse_parameters_dict(parameters, remaining_time, using_cyclic_pricing));
            new_routes = full_pricing_problems_basic(
                convex_dual_solution,
                instance,
                vehicle_order,
                parameters.solver_objective_mode,
                using_cyclic_pricing,
                n_ressources_dominance
            );
        } else if (parameters.pricing_function == PRICING_DIVERSIFICATION){
            Parameters::setParametersFromDict(pathwyse_parameters_dict(parameters, remaining_time, using_cyclic_pricing));
            new_routes = full_pricing_problems_diversification(
                convex_dual_solution,
                instance,
                vehicle_order,
                parameters.solver_objective_mode,
                using_cyclic_pricing,
                n_ressources_dominance,
                iteration
            );
        } else if (parameters.pricing_function == PRICING_PA_BASIC){
            new_routes = full_pricing_problems_basic_pulse(
                convex_dual_solution,
                instance,
                vehicle_order,
                parameters.solver_objective_mode,
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
                parameters.solver_objective_mode,
                parameters.delta,
                parameters.solution_pool_size,
                parameters.pricing_verbose
            );
        }
        auto end_pricing = chrono::steady_clock::now();
        int diff_pricing = chrono::duration_cast<chrono::milliseconds>(end_pricing - start_pricing).count();
        pricing_time += diff_pricing;

        // We add the new routes to the global routes vector, only if the time limit is not reached
        double min_reduced_cost = std::numeric_limits<double>::infinity();
        double max_reduced_cost = - std::numeric_limits<double>::infinity();
        if (pricing_time + master_time < S_TO_MS * parameters.time_limit) {
            if (parameters.solver_objective_mode == SolverMode::BIG_M_FORMULATION_COVERAGE){
                for (Route& new_route : new_routes){
                    max_reduced_cost = std::max(max_reduced_cost, new_route.reduced_cost);
                    if (new_route.reduced_cost > parameters.reduced_cost_threshold){
                        add_route(model, new_route, instance, route_vars, intervention_ctrs, vehicle_ctrs, parameters.solver_objective_mode);
                        n_added_routes++;
                        routes.push_back(new_route);

                    }
                }
            } else {
                for (Route& new_route : new_routes){
                    min_reduced_cost = std::min(min_reduced_cost, new_route.reduced_cost);
                    if (new_route.reduced_cost < - parameters.reduced_cost_threshold){
                        add_route(model, new_route, instance, route_vars, intervention_ctrs, vehicle_ctrs, parameters.solver_objective_mode);
                        n_added_routes++;
                        routes.push_back(new_route);
                    }
                }
            }  
        }

        if (parameters.verbose) {
            cout << "Pricing sub problems solved in " << diff_pricing << " ms - Added " << n_added_routes << " routes";
            if (parameters.solver_objective_mode == SolverMode::BIG_M_FORMULATION_COVERAGE){
                cout << " - Max RC : " << setprecision(8) << max_reduced_cost << "\n";
            } else {
                cout << " - Min RC : " << setprecision(8) << min_reduced_cost << "\n";
            }
        }

        // ----------------- Master problem -----------------

        auto start = chrono::steady_clock::now();
        int status = solve_model(model);
        solution = extract_solution(model, route_vars, intervention_ctrs, vehicle_ctrs);
        auto end = chrono::steady_clock::now();
        int diff = chrono::duration_cast<chrono::milliseconds>(end - start).count();
        DualSolution& dual_solution = solution.dual_solution;

        // ----------------- Intermediate integer solutions -----------------

        if (parameters.compute_intermediate_integer_solutions && iteration % 5 == 0 && iteration > 0){
            cout << "Solving intermediary integer model at iteration " << iteration << endl;
            intermediary_integer_solution = solve_intermediary_integer_model(model, route_vars, postpone_vars, parameters.time_limit);

            intermediary_integer_routes = routes;
            if (!(parameters.solver_objective_mode == SolverMode::BIG_M_FORMULATION_COVERAGE)){
                // Repair the integer solution
                repair_routes(intermediary_integer_routes, intermediary_integer_solution, instance);
            }
        }

        // ----------------- Objective tracking -----------------

        objective_values.push_back(solution.objective_value); 
        solution_costs.push_back(relaxed_solution_cost(solution, routes));
        covered_interventions.push_back(count_covered_interventions(solution, routes, instance));
        time_points.push_back((master_time + pricing_time) / 1000.0);

        if (parameters.compute_intermediate_integer_solutions){
            // We add them at every time point even when no integer solution is computed
            // This way we can just plot against the time points
            integer_objective_values.push_back(intermediary_integer_solution.objective_value);
            integer_covered_interventions.push_back(count_covered_interventions(intermediary_integer_solution, intermediary_integer_routes, instance));
            integer_solution_costs.push_back(integer_solution_cost(intermediary_integer_solution, intermediary_integer_routes));
        }

        if (parameters.verbose) {
            cout << "Iteration " << iteration << " - Objective value : " << solution.objective_value;
            cout << " - Master problem solved in " << diff << " ms \n";
            cout << "Number of interventions covered : " << setprecision(3) << count_covered_interventions(solution, routes, instance);
            std::pair<double, int> used_vehicles = count_used_vehicles(solution, routes, instance);
            cout << " - Number of vehicles used : " << used_vehicles.first << " - Unique vehicles used : " << used_vehicles.second << "\n";
        }
        master_time += diff;

        // ----------------- Stop conditions -----------------

        if (master_time + pricing_time >= S_TO_MS * parameters.time_limit){
            cout << "-----------------------------------" << endl;
            cout << "Time limit reached" << endl;
            stop = true;
            continue;
        }

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
            cout << "Found no new route to add" << endl;
            stop = true;
        }
        // Count the number of consecutive non improvement
        if (parameters.solver_objective_mode == SolverMode::BIG_M_FORMULATION_COVERAGE 
            && solution.objective_value <= previous_solution_objective){
            consecutive_non_improvement++;
        } else if (!(parameters.solver_objective_mode == SolverMode::BIG_M_FORMULATION_COVERAGE) 
            && solution.objective_value >= previous_solution_objective){
            consecutive_non_improvement++;
        } else {
            consecutive_non_improvement = 0;
        }
        if (consecutive_non_improvement >= parameters.max_consecutive_non_improvement){
            cout << "-----------------------------------" << endl;
            cout << "Stopped after " << parameters.max_consecutive_non_improvement << " iterations without improvement" << endl;
            stop = true;
        }
        if (parameters.solver_objective_mode == SolverMode::DURATION_ONLY 
            && solution.objective_value <= duration_lower_bound){
            cout << "-----------------------------------" << endl;
            cout << "Reached the pre-processed minimum outsourced duration" << endl;
            stop = true;
        }
        previous_solution_objective = solution.objective_value;
        previous_dual_solution = dual_solution;
        iteration++;
    }

    // ----------------- End of the column generation -----------------
    cout << "End of the column generation after " << iteration << " iterations" << endl;
    // Convert the value from the minimum formulation to the maximum formulation
    double total_outsource_cost = 0;
    for (int i = 0; i < instance.number_interventions; i++){
        total_outsource_cost += instance.nodes[i].duration * instance.M;
    }
    if (parameters.solver_objective_mode == SolverMode::BIG_M_FORMULATION_COVERAGE){
        cout << "Relaxed RMP minimization objective value : " << setprecision(8) << total_outsource_cost - solution.objective_value << endl;
        cout << "Relaxed RMP maximization objective value : " << setprecision(8) << solution.objective_value << endl;
    } else if (parameters.solver_objective_mode == SolverMode::BIG_M_FORMULATION_OUTSOURCE){
        cout << "Relaxed RMP minimization objective value : " << setprecision(8) << solution.objective_value << endl;
        cout << "Relaxed RMP maximization objective value : " << setprecision(8) << total_outsource_cost - solution.objective_value << endl;
    } else if (parameters.solver_objective_mode == SolverMode::SOLUTION_MINIMISATION){
        cout << "Relaxed RMP solution cost : " << setprecision(8) << solution.objective_value << endl;
    } else if (parameters.solver_objective_mode == SolverMode::DURATION_ONLY){
        cout << "Relaxed RMP outsourced duration : " << setprecision(8) << solution.objective_value << endl;
    }
    

    // ----------------- Integer solution -----------------

    int integer_time = 0;
    auto integer_solution = IntegerSolution{};
    if (parameters.compute_integer_solution) {
        auto start_integer = chrono::steady_clock::now();
        integer_solution = solve_intermediary_integer_model(model, route_vars, postpone_vars, parameters.time_limit);
        // If applicable, repair the integer solution
        if (!(parameters.solver_objective_mode == SolverMode::BIG_M_FORMULATION_COVERAGE)){
            repair_routes(routes, integer_solution, instance);
        }
        auto end_integer = chrono::steady_clock::now();
        integer_time = chrono::duration_cast<chrono::milliseconds>(end_integer - start_integer).count();
        
        // Objective values tracking
        double integer_max_objective;
        double integer_min_objective;
        if (parameters.solver_objective_mode == SolverMode::BIG_M_FORMULATION_COVERAGE){
            cout << "Integer RMP minimization objective value : " << setprecision(8) << total_outsource_cost - integer_solution.objective_value;
            double minimisation_gap = (solution.objective_value - integer_solution.objective_value) / (total_outsource_cost - integer_solution.objective_value);
            cout << " - Minimisation integrality Gap : " << setprecision(3) << minimisation_gap * 100 << "%" << endl;
        } else if (parameters.solver_objective_mode == SolverMode::BIG_M_FORMULATION_OUTSOURCE){
            cout << "Integer RMP minimization objective value : " << setprecision(8) << integer_solution.objective_value;
            double minimisation_gap = (integer_solution.objective_value - solution.objective_value) / integer_solution.objective_value;
            cout << " - Minimisation integrality Gap : " << setprecision(3) << minimisation_gap * 100 << "%" << endl;
        } else if (parameters.solver_objective_mode == SolverMode::SOLUTION_MINIMISATION){
            cout << "Integer RMP solution cost : " << setprecision(8)  << endl;
        } else if (parameters.solver_objective_mode == SolverMode::DURATION_ONLY){
            cout << "Integer RMP covered duration : " << setprecision(8) << time_spent_working(integer_solution, routes, instance) << endl;
        }

    }

    // ----------------- Data analysis -----------------
    int n_routes_generated = routes.size();
    cout << "Number of routes generated : " << n_routes_generated;
    cout << " - Number of duplicate routes : " << count_routes_with_duplicates(routes);
    cout << " - Average time to generate a route : " << pricing_time / n_routes_generated << " ms" << endl;
    
    int elapsed_time = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - procedure_start).count();
    // Print the time it took to solve the master problem
    cout << "-----------------------------------" << endl;
    cout << "Total time spent solving the master problem : " << master_time << " ms" << endl;
    cout << "Total time spent solving the pricing problems : " << pricing_time << " ms - Average : " << pricing_time / iteration << " ms" << endl;
    cout << "Total time spent solving the integer problem : " << integer_time << " ms" << endl;
    cout << "Total elapsed time : " << elapsed_time << " ms" << endl;

    if (integer_solution.is_feasible){
        cout << "-----------------------------------" << endl;
        cout << "Full analysis of the integer solution" << endl;
        full_analysis(integer_solution, routes, instance);
    }

    cout << "-----------------------------------" << endl;

    return CGResult{
        solution,
        integer_solution,
        iteration,
        master_time,
        pricing_time,
        integer_time,
        objective_values,
        integer_objective_values,
        solution_costs,
        integer_solution_costs,
        covered_interventions,
        integer_covered_interventions,
        time_points
    };
}
