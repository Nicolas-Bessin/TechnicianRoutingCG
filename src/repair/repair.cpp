#include "repair.h"

#include <vector>
#include <numeric>
#include <algorithm>



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

std::vector<Route> repair_routes(const std::vector<Route>& routes, const IntegerSolution& solution, const Instance& instance) {
    using std::vector;

    // First, create a vector with only the routes that are used
    vector<Route> used_routes;
    for (int i = 0; i < routes.size(); i++){
        if (solution.coefficients[i] > 0){
            used_routes.push_back(routes[i]);
        }
    }


    // Then, for every intervention, build a vector of the routes that cover it
    int n_nodes = used_routes.at(0).is_in_route.size();
    vector<vector<int>> routes_covering_intervention = vector<vector<int>>(n_nodes);
    for (int r = 0; r < used_routes.size(); r++){
        for (int i = 0; i < used_routes[r].is_in_route.size(); i++){
            if (used_routes[r].is_in_route[i] > 0){
                routes_covering_intervention[i].push_back(r);
            }
        }
    }

    // Finally, process the routes that cover it and remove the intervention where adventageous
    for (int i = 0; i < n_nodes; i++){
        if (routes_covering_intervention[i].size() > 1){
            // If there are multiple routes covering the intervention, compute the "delta" of removing the intervention from each route
            vector<double> delta = vector<double>(routes_covering_intervention[i].size());
            for (int r = 0; r < routes_covering_intervention[i].size(); r++){
                Route route = used_routes[routes_covering_intervention[i][r]];
                delta[r] = compute_delta(route, i , instance);
            }
            // Sort the routes by the delta
            vector<int> indexes(routes_covering_intervention[i].size());
            std::iota(indexes.begin(), indexes.end(), 0);
            std::sort(indexes.begin(), indexes.end(), [&delta](int i1, int i2) {return delta[i1] < delta[i2];});

            // Remove the intervention from all the routes except the one with the highest delta
            for (int j = 0; j < indexes.size() - 1; j++){
                int r = routes_covering_intervention[i][indexes[j]];
                delete_intervention(used_routes[r], i, instance);
            }
        }
    }

    // Remove the routes that are empty
    for (auto it = used_routes.begin(); it != used_routes.end();){
        if (it->id_sequence.size() == 2){
            it = used_routes.erase(it);
        } else {
            it++;
        }
    }

    return used_routes;
}