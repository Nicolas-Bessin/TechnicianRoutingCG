#include "solution.h"
#include <math.h>
#include <algorithm>

using std::min;

// Check wether two consecutive master solutions are equal (i.e whether their non zero coefficients are the same)
bool operator==(const MasterSolution& lhs, const MasterSolution& rhs){
    // Check the objective value
    if (fabs(lhs.objective_value - rhs.objective_value) > 0.0001){
        return false;
    }
    // Check the length of the coefficients
    // Check the coefficients themselves
    int size = min(lhs.coefficients.size(), rhs.coefficients.size());
    for (int i = 0; i < size; i++){
        if (lhs.coefficients[i] != rhs.coefficients[i]){
            return false;
        }
    }
    return true;
}

// Checks if two routes are equal
// That is, if they have :
// - the same vehicle_id
// - the same sequence vector
// - the same start times
bool operator==(const Route& lhs, const Route& rhs){
    // Check the vehicle_id
    if (lhs.vehicle_id != rhs.vehicle_id){
        return false;
    }
    //Check the length of the id_sequence
    if (lhs.id_sequence.size() != rhs.id_sequence.size()){
        return false;
    }
    // Check the sequences themselves
    for (int i = 0; i < lhs.id_sequence.size(); i++){
        if (lhs.id_sequence[i] != rhs.id_sequence[i]){
            return false;
        }
    }
    // Check the start times
    for (int i = 0; i < lhs.start_times.size(); i++){
        if (fabs(lhs.start_times[i] - rhs.start_times[i]) > 0.0001){
            return false;
        }
    }

    return true;
}


bool operator<(const Route& lhs, const Route& rhs){
    // Check the vehicle_id
    if (lhs.vehicle_id < rhs.vehicle_id){
        return true;
    }
    if (lhs.vehicle_id > rhs.vehicle_id){
        return false;
    }
    //Check the length of the id_sequence
    if (lhs.id_sequence.size() < rhs.id_sequence.size()){
        return true;
    }
    if (lhs.id_sequence.size() > rhs.id_sequence.size()){
        return false;
    }
    // Check the sequences themselves
    for (int i = 0; i < lhs.id_sequence.size(); i++){
        if (lhs.id_sequence[i] < rhs.id_sequence[i]){
            return true;
        }
        if (lhs.id_sequence[i] > rhs.id_sequence[i]){
            return false;
        }
    }
    // Check the start times
    for (int i = 0; i < lhs.start_times.size(); i++){
        if (lhs.start_times[i] < rhs.start_times[i]){
            return true;
        }
        if (lhs.start_times[i] > rhs.start_times[i]){
            return false;
        }
    }
    // If we reach this point, the two routes are equal
    return false;
}



double compute_reduced_cost(const Route& route, const std::vector<double>& alphas, double beta, const Instance& instance) {
    const Vehicle& vehicle = instance.vehicles[route.vehicle_id];
    // Compute the reduced cost along the route by going through the nodes
    double reduced_cost;
    int tour_length = route.id_sequence.size();

    // Initialize the reduced cost with the cost of the vehicle
    reduced_cost = - beta - vehicle.cost;

    if (tour_length <= 1) {
        return reduced_cost;
    }
    // Add the cost of the edge from the warehouse to the first node
    const Node& depot = instance.nodes[route.id_sequence[0]];
    const Node& first_inervention = instance.nodes[route.id_sequence[1]];
    double distance = metric(depot, first_inervention, instance.distance_matrix);
    reduced_cost -= distance * instance.cost_per_km;

    // Go through the consecutive interventions in the route
    for (int i = 1; i < tour_length - 1; i++) {
        const Node& node = instance.nodes[route.id_sequence[i]];
        const Node& next_node = instance.nodes[route.id_sequence[i + 1]];
        // Add the cost associated with the node
        reduced_cost += instance.M * node.duration - alphas.at(route.id_sequence[i]);
        // Then add the cost associated with the arc
        double distance = metric(node, next_node, instance.distance_matrix);
        reduced_cost -= distance * instance.cost_per_km;
    }
    // Add the cost associated with the last node
    const Node& last_node = instance.nodes[route.id_sequence[tour_length - 1]];
    reduced_cost += instance.M * last_node.duration; // should add 0

    return reduced_cost;
}