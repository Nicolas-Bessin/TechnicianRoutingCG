#include "genetic_solver.h"

#include <vector>
#include <array>
#include <random>
#include <bits/stdc++.h>

double evaluate(const GeneticSolution& solution, const Instance& instance) {
    double objective = 0;
    for (const Route& route : solution.routes){
        objective += instance.M * route.total_duration - route.total_cost;
    }
    return objective;
}


GeneticSolution crossover(const GeneticSolution& parent1, const GeneticSolution& parent2, const Instance& instance) {
    using std::vector;
    // Enumerate through all the vehicles in the instance, and if two are available, pick a random parent to get the route from
    GeneticSolution child;
    for (int v = 0; v < instance.number_vehicles; v++){
        // If both parents have the vehicle, pick one at random
        if (parent1.vehicle_to_route.count(v) && parent2.vehicle_to_route.count(v)){
            // Generate a random number between 0 and 1
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0, 1);
            double random = dis(gen);
            if (random < 0.5){
                child.routes.push_back(parent1.routes[parent1.vehicle_to_route.at(v)]);
                child.vehicle_to_route[v] = child.routes.size() - 1;
            } else {
                child.routes.push_back(parent2.routes[parent2.vehicle_to_route.at(v)]);
                child.vehicle_to_route[v] = child.routes.size() - 1;
            }
        } else if (parent1.vehicle_to_route.count(v)){  
            child.routes.push_back(parent1.routes[parent1.vehicle_to_route.at(v)]);
            child.vehicle_to_route[v] = child.routes.size() - 1;
        } else if (parent2.vehicle_to_route.count(v)){
            child.routes.push_back(parent2.routes[parent2.vehicle_to_route.at(v)]);
            child.vehicle_to_route[v] = child.routes.size() - 1;
        }
    }
    // First thing : we need to repair the solution
    repair(child, instance);
    // Evaluate the objective value of the child
    child.objective = evaluate(child, instance);

    return child;
}


// @brief Computes the delta in the route's length if we remove a given intervention
// @param route The route from which we want to remove the intervention
// @param intervention The intervention we want to remove
// @param instance The instance of the problem
// The delta is defined as : cost(route) - cost(route without intervention)
// Thus, the bigger the delta, the more advantageous it is to remove the intervention
double compute_delta(const Route& route, int intervention, const Instance& instance){
    // First, we need to find the intervention in the route
    auto it = std::find(route.id_sequence.begin(), route.id_sequence.end(), intervention);
    // Since the sequence necessarily begins and ends with the depot, we know that it +- 1 is always a valid index
    int previous_intervention = *(it - 1);
    int next_intervention = *(it + 1);
    // We can now compute the delta
    double length_including = instance.distance_matrix[previous_intervention][intervention] + instance.distance_matrix[intervention][next_intervention];
    double length_excluding = instance.distance_matrix[previous_intervention][next_intervention];
    return length_including - length_excluding;
}


// @brief Deletes a given intervention from a route
// @param route The route from which we want to remove the intervention
// @param intervention The intervention we want to remove
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

    

}

void repair(GeneticSolution& solution, const Instance& instance) {
    // Each intervention can be covered at most twice
    // This is because each parent represents a feasible solution - so each intervention is covered at most once per parent
    // Thus among the two parents, there are at most two routes that cover the same intervention
    
    // First step is seeing which interventions are covered twice, and which routes they are covered by
    std::map<int, std::array<int, 2>> intervention_to_routes;
    for (int i = 0; i < instance.number_interventions; i++){
        intervention_to_routes[i] = {-1, -1};
        for (int r = 0; r < solution.routes.size(); r++){
            if (solution.routes[r].is_in_route[i] == 1){
                if (intervention_to_routes[i][0] == -1){
                    intervention_to_routes[i][0] = r;
                } else {
                    intervention_to_routes[i][1] = r;
                }
            }
        }
    }

    // Now we go through the interventions covered twice and see which one is the most advantageous to remove
    // For now, we do our removal by removing where the relative increase in route objective coefficient is the biggest
    for (const auto& [intervention, indexes] : intervention_to_routes){
        if (indexes[0] == -1 || indexes[1] == -1) continue;
        // Get the delta in route length if we remove the intervention from each route
        double delta1 = compute_delta(solution.routes[indexes[0]], intervention, instance);
        double delta2 = compute_delta(solution.routes[indexes[1]], intervention, instance);
        // Get the biggest relative increase in route objective coefficient 
        // Equivalent to the biggest delta in route length
        double relative1 = delta1 / count_route_kilometres(solution.routes[indexes[0]], instance);
        double relative2 = delta2 / count_route_kilometres(solution.routes[indexes[1]], instance);
    }
}