#include "solution.h"
#include "constants.h"
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



// Check that a route is feasible :
// - The route starts and ends at the depot
// - The route respects the time windows of the interventions
// - The route respects the capacities of the vehicles
bool is_route_feasible(const Route& route, const Instance& instance) {
    using std::cout, std::endl;
    using std::map, std::string;

    int vehicle_id = route.vehicle_id;
    const Vehicle& vehicle = instance.vehicles.at(vehicle_id);
    int depot_id = vehicle.depot;
    // If the route is empty, it is feasible
    if (route.id_sequence.size() == 0) {
        return true;
    }
    // Check that the route starts and ends at the depot
    if (route.id_sequence.front() != depot_id) {
        cout << "Route does not start at the depot" << endl;
        return false;
    }
    if (route.id_sequence.back() != depot_id) {
        cout << "Route does not end at the depot" << endl;
        return false;
    }

    // We now go through the route step by step to check the time windows and the capacities
    int route_length = route.id_sequence.size();
    double current_time = 0;
    map<string, int> consummed_capacities;
    // Go through every consecutive intervention in the route
    for (int i = 0; i < route_length - 1; i++) {
        // Check that the time window is respected
        int intervention_id = route.id_sequence.at(i);
        const Node& intervention = instance.nodes.at(route.id_sequence.at(i));
        double duration = intervention.duration;
        // Check that the start time is the current time 
        // (for the first node which is the depot, the start time will hold the arrival time at the end of the day)
        if (i > 0 && route.start_times[intervention_id] != current_time) {
            cout << "Start time of intervention " << i << " is not the time of arrival" << endl;
            cout << "Start time: " << route.start_times[intervention_id] << " Current time: " << current_time << endl;
            return false;
        }
        // Check that the time window is respected (with the way we built the solution, we can't arrive too early)
        if (current_time < intervention.start_window) {
            cout << "Intervention " << i << " starts too early" << endl;
            return false;
        }
        if (current_time + duration > intervention.end_window) {
            cout << "Intervention " << i << " ends too late : " << current_time + duration << " > " << intervention.end_window << endl;
            return false;
        }
        // Check wether the lunch break is respected
        if (intervention.is_ambiguous && current_time < MID_DAY && current_time + duration > MID_DAY) {
            cout << "Intervention " << intervention_id << " ends after the lunch break : start = " << current_time << " end = " << current_time + duration << endl;
            return false;
        }
        // Update the quantities consummed
        for (auto& [key, value] : intervention.quantities) {
            consummed_capacities[key] += value;
        }
        // Update the current time
        const Node& next_intervention = instance.nodes[route.id_sequence[i + 1]];
        double travel_time = metric(intervention, next_intervention, instance.time_matrix);
        current_time += duration + travel_time;
        // If we arrive too early, we will wait
        if (current_time < next_intervention.start_window) {
            current_time = next_intervention.start_window;
        }
    }
    // Check the final intervention
    const Node& final_intervention = instance.nodes[route.id_sequence.back()];
    int final_intervention_id = route.id_sequence.back();
    if (route.start_times[final_intervention_id] != current_time) {
        cout << "Start time of the final intervention is not the time of arrival" << endl;
        return false;
    }
    if (current_time < final_intervention.start_window) {
        cout << "Final intervention starts too early" << endl;
        return false;
    }
    if (current_time + final_intervention.duration > final_intervention.end_window) {
        cout << "Final intervention ends too late : " << current_time + final_intervention.duration << " > " << final_intervention.end_window << endl;
        return false;
    }
    // Finally, check that the capacities are respected
    for (auto& [key, value] : consummed_capacities) {
        // If the key is not in the capacities, we don't care
        if (vehicle.capacities.find(key) == vehicle.capacities.end()) {
            continue;
        }
        if (value > vehicle.capacities.at(key)) {
            cout << "Capacity " << key << " is exceeded" << endl;
            return false;
        }
    }

    return true;
}


