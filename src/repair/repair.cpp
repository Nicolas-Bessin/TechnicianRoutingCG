#include "repair.h"

#include <vector>
#include <numeric>
#include <algorithm>

#include "master_problem/master_solver.h"



double compute_delta(const Route& route, int intervention, const Instance& instance){
    // First, we need to find the intervention in the route
    auto it = std::find(route.id_sequence.begin(), route.id_sequence.end(), intervention);
    // Since the sequence necessarily begins and ends with the depot, we know that it +- 1 is always a valid index
    int previous_intervention = *(it - 1);
    int next_intervention = *(it + 1);
    // We can now compute the delta
    double length_including = instance.distance_matrix[previous_intervention][intervention] + instance.distance_matrix[intervention][next_intervention];
    double length_excluding = instance.distance_matrix[previous_intervention][next_intervention];
    return (length_including - length_excluding) * instance.cost_per_km;
}


void delete_intervention(Route& route, int intervention, const Instance& instance){
    // First, we need to find the intervention in the route
    auto it = std::find(route.id_sequence.begin(), route.id_sequence.end(), intervention);
    // Since the sequence necessarily begins and ends with the depot, we know that it +- 1 is always a valid index
    int previous_intervention = *(it - 1);
    int next_intervention = *(it + 1);
    // We can now delete the intervention
    route.id_sequence.erase(it);
    route.is_in_route[intervention] = 0;
    // Update the edge matrix
    route.route_edges[previous_intervention][intervention] = 0;
    route.route_edges[intervention][next_intervention] = 0;
    route.route_edges[previous_intervention][next_intervention] = 1;
    // Update the total duration of the route
    route.total_duration -= instance.nodes[intervention].duration;
    // Update the total cost of the route
    int distance_deleted = instance.distance_matrix[previous_intervention][intervention] + instance.distance_matrix[intervention][next_intervention];
    int distance_added = instance.distance_matrix[previous_intervention][next_intervention];
    route.total_cost += (distance_added - distance_deleted) * instance.cost_per_km;
}

void repair_routes(std::vector<Route>& routes, IntegerSolution& solution, const Instance& instance) {
    using std::vector;

    // For every intervention, build a vector of the routes that cover it
    int n_interventions = instance.number_interventions;
    vector<vector<int>> routes_covering_intervention = vector<vector<int>>(n_interventions);
    for (int r = 0; r < routes.size(); r++){
        for (int i = 0; i < n_interventions; i++){
            if (solution.coefficients[r] > 0 && routes[r].is_in_route[i] > 0){
                routes_covering_intervention[i].push_back(r);
            }
        }
    }

    // Finally, process the routes that cover it and remove the intervention where adventageous
    for (int i = 0; i < n_interventions; i++){
        if (routes_covering_intervention[i].size() <= 1){
            continue;
        }
        // If there are multiple routes covering the intervention, compute the "delta" of removing the intervention from each route
        vector<double> delta = vector<double>(routes_covering_intervention[i].size());
        for (int r = 0; r < routes_covering_intervention[i].size(); r++){
            Route& route = routes[routes_covering_intervention[i][r]];
            delta[r] = compute_delta(route, i , instance);
        }
        // Sort the routes by the delta
        vector<int> indexes(routes_covering_intervention[i].size());
        std::iota(indexes.begin(), indexes.end(), 0);
        std::sort(indexes.begin(), indexes.end(), [&delta](int i1, int i2) {return delta[i1] > delta[i2];});

        // Remove the intervention from all the routes except the one with the highest delta
        for (int j = 0; j < indexes.size() - 1; j++){
            int r = routes_covering_intervention[i][indexes[j]];
            delete_intervention(routes[r], i, instance);
        }
    }

    // Remove the routes that are empty
    for (int r = 0; r < routes.size(); r++){
        if (routes[r].id_sequence.size() == 2){
            solution.coefficients[r] = 0;
        }
    }

    // Update the cost of the solution
    solution.objective_value = compute_integer_objective(solution, routes, instance, true);
}

std::pair<IntegerSolution, std::vector<Route>> compute_repaired_solution(const std::vector<Route>& routes, const IntegerSolution& solution, const Instance& instance){
    using std::vector;

    // Create a vector containing only the routes used in the initial solution
    vector<Route> used_routes;
    IntegerSolution repaired_solution = AllOnesSolution(used_routes.size());
    for (int r = 0; r < routes.size(); r++){
        if (solution.coefficients[r] > 0){
            used_routes.push_back(routes[r]);
        }
    }

    // For every intervention, build a vector of the routes that cover it
    int n_interventions = instance.number_interventions;
    vector<vector<int>> routes_covering_intervention = vector<vector<int>>(n_interventions);
    for (int r = 0; r < used_routes.size(); r++){
        for (int i = 0; i < n_interventions; i++){
            if (solution.coefficients[r] > 0 && used_routes[r].is_in_route[i] > 0){
                routes_covering_intervention[i].push_back(r);
            }
        }
    }

    // Finally, process the routes that cover it and remove the intervention where adventageous
    for (int i = 0; i < n_interventions; i++){
        if (routes_covering_intervention[i].size() <= 1){
            continue;
        }
        // If there are multiple routes covering the intervention, compute the "delta" of removing the intervention from each route
        vector<double> delta = vector<double>(routes_covering_intervention[i].size());
        for (int r = 0; r < routes_covering_intervention[i].size(); r++){
            Route& route = used_routes[routes_covering_intervention[i][r]];
            delta[r] = compute_delta(route, i , instance);
        }
        // Sort the routes by the delta
        vector<int> indexes(routes_covering_intervention[i].size());
        std::iota(indexes.begin(), indexes.end(), 0);
        std::sort(indexes.begin(), indexes.end(), [&delta](int i1, int i2) {return delta[i1] > delta[i2];});

        // Remove the intervention from all the routes except the one with the highest delta
        for (int j = 0; j < indexes.size() - 1; j++){
            int r = routes_covering_intervention[i][indexes[j]];
            delete_intervention(used_routes[r], i, instance);
        }
    }

    // Remove the routes that are empty
    for (int r = 0; r < used_routes.size(); r++){
        if (routes[r].id_sequence.size() == 2){
            repaired_solution.coefficients[r] = 0;
        }
    }

    // Update the cost of the solution
    repaired_solution.objective_value = compute_integer_objective(solution, routes, instance, true);

    return std::make_pair(repaired_solution, used_routes);
}