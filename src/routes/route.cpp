#include "route.h"

#include "master_problem/master.h"
#include "instance/constants.h"

#include <math.h>
#include <fstream>
#include <iostream>
#include <algorithm>

#include "../../nlohmann/json.hpp"
#include "gurobi_c++.h"



Route EmptyRoute(int n_nodes) {
    Route route{};
    route.is_in_route = std::vector<int>(n_nodes, 0);
    route.route_edges = std::vector<std::vector<int>>(n_nodes, std::vector<int>(n_nodes, 0));
    return route;
}


Route convert_sequence_to_route(double rc, const std::vector<int> & sequence, const Instance& instance, const Vehicle& vehicle) {
    using std::vector;
    // Get all the info we need to build a Route object
    double total_cost = vehicle.cost;
    int total_duration = 0;
    vector<int> id_sequence;
    vector<int> is_in_route(instance.nodes.size(), 0);
    vector<vector<int>> route_edges(instance.nodes.size(), vector<int>(instance.nodes.size(), 0));


    for (int i = 0; i < sequence.size() - 1; i++) {
        int true_i = i == 0 ? vehicle.depot : vehicle.interventions[sequence[i]];
        int true_j = i+1 == sequence.size()-1 ? vehicle.depot : vehicle.interventions[sequence[i + 1]];
        // Update the sequence of interventions
        id_sequence.push_back(true_i);
        // Update the edge matrix
        route_edges[true_i][true_j] = 1;
        // Update the is_in_route
        is_in_route[true_i] = 1;
        // Get the duration, and distance between the two interventions
        int duration = instance.nodes[true_i].duration;
        int distance = instance.distance_matrix[true_i][true_j];
        // Update the running total cost & duration
        total_cost += instance.cost_per_km * distance;
        total_duration += duration;        
    }
    // Add the checks related to the last intervention
    int true_last = vehicle.depot;
    id_sequence.push_back(true_last);
    is_in_route[true_last] = 1;

    // Create the Route object
    return Route{
        vehicle.id,
        total_cost,
        rc,
        total_duration,
        id_sequence,
        is_in_route,
        route_edges
    };
}

// Checks if two routes are equal
// That is, if they have :
// - the same vehicle_id
// - the same sequence vector
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

    return true;
}

double count_route_kilometres(const Route& route, const Instance& instance) {
    double total_distance = 0;
    for (int i = 0; i < route.id_sequence.size() - 1; i++) {
        int id1 = route.id_sequence[i];
        int id2 = route.id_sequence[i + 1];
        total_distance += instance.distance_matrix[id1][id2];
    }
    return total_distance;
}

int count_route_duration(const Route& route, const Instance& instance) {
    if (route.id_sequence.size() <= 2) {
        return 0;
    }
    int total_duration = 0;
    for (int i = 1; i < route.id_sequence.size() - 1; i++) {
        int id = route.id_sequence[i];
        total_duration += instance.nodes.at(id).duration;
    }
    return total_duration;
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
    double distance = instance.distance_matrix[route.id_sequence[0]][route.id_sequence[1]];
    reduced_cost -= distance * instance.cost_per_km;

    // Go through the consecutive interventions in the route
    for (int i = 1; i < tour_length - 1; i++) {
        int node_id = route.id_sequence[i];
        const Node& node = instance.nodes[node_id];
        int next_node_id = route.id_sequence[i + 1];
        // Add the cost associated with the node
        reduced_cost += instance.M * node.duration - alphas.at(node_id);
        // Then add the cost associated with the arc
        double distance = instance.distance_matrix[node_id][next_node_id];
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
    int current_time = 0;
    map<string, int> consummed_capacities;
    // Go through every consecutive intervention in the route
    for (int i = 0; i < route_length - 1; i++) {
        int intervention_id = route.id_sequence.at(i);
        const Node& intervention = instance.nodes.at(intervention_id);
        // Check that the skills are respected
        bool can_do = can_do_intervention(intervention, vehicle);
        if (!can_do) {
            cout << "Vehicle " << vehicle_id << " cannot do intervention " << intervention_id << endl;
            return false;
        }
        // Check that the time window is respected
        double duration = intervention.duration;
        // Check that intervention ends before the end of its time window
        if (current_time + duration > intervention.end_window) {
            cout << "Intervention" << intervention_id << " ends too late : " << current_time + duration << " > " << intervention.end_window << endl;
            return false;
        }
        // Update the quantities consummed
        for (auto& [key, value] : intervention.quantities) {
            consummed_capacities[key] += value;
        }
        // Update the current time
        int next_intervention_id = route.id_sequence[i + 1];
        const Node& next_intervention = instance.nodes[next_intervention_id];
        double travel_time = instance.time_matrix[intervention_id][next_intervention_id];
        current_time += duration + travel_time;
        // If we arrive too early, we will wait
        if (current_time < next_intervention.start_window) {
            current_time = next_intervention.start_window;
        }
        // If we need to wait out of the lunch break, we wait
        if (next_intervention.is_ambiguous && current_time < MID_DAY && current_time + next_intervention.duration > MID_DAY) {
            current_time = MID_DAY;
        }
    }
    // Check the final intervention, we only have to check the end window
    if (current_time > END_DAY) {
        cout << "Final intervention ends too late : " << current_time << " > " << END_DAY << endl;
        return false;
    }
    // Finally, check that the capacities are respected
    for (const auto& [key, value] : consummed_capacities) {
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

std::vector<int> compute_start_times(const Route& route, const Instance& instance) {
    // We go through the route and compute the start times of the interventions
    std::vector<int> start_times;
    int current_time = 0;
    for (int i = 0; i < route.id_sequence.size() - 1; i++) {
        // We just arrived at the intervention - push the current time
        start_times.push_back(current_time);
        // Update the current time
        int intervention_id = route.id_sequence[i];
        const Node& intervention = instance.nodes[intervention_id];
        current_time += intervention.duration;
        // Add the travel time to the next intervention
        int next_intervention_id = route.id_sequence[i + 1];
        const Node& next_intervention = instance.nodes[next_intervention_id];
        current_time += instance.time_matrix[intervention_id][next_intervention_id];
        // And the eventual waiting time
        current_time = std::max(current_time, next_intervention.start_window);
        // And the eventual waiting out of the lunch break
        if (next_intervention.is_ambiguous && current_time < MID_DAY &&  current_time + next_intervention.duration > MID_DAY) {
            current_time = MID_DAY;
        }
    }
    // Treat the last intervention
    start_times.push_back(current_time);

    return start_times;
}

int compute_total_waiting_time(const Route& route, const Instance& instance) {
    // We go through the route and compute the total waiting time
    int total_waiting_time = 0;
    int current_time = 0;
    for (int i = 0; i < route.id_sequence.size() - 1; i++) {
        // Update the current time
        int intervention_id = route.id_sequence[i];
        const Node& intervention = instance.nodes[intervention_id];
        current_time += intervention.duration;
        // Add the travel time to the next intervention
        int next_intervention_id = route.id_sequence[i + 1];
        const Node& next_intervention = instance.nodes[next_intervention_id];
        current_time += instance.time_matrix[intervention_id][next_intervention_id];
        // And the eventual waiting time
        total_waiting_time += std::max(0, next_intervention.start_window - current_time);
        current_time = std::max(current_time, next_intervention.start_window);
        // And the eventual waiting out of the lunch break
        if (next_intervention.is_ambiguous && current_time < MID_DAY &&  current_time + next_intervention.duration > MID_DAY) {
            total_waiting_time += MID_DAY - current_time;
            current_time = MID_DAY;
        }
    }
    return total_waiting_time;
}

int compute_total_travelling_time(const Route& route, const Instance& instance) {
    // We go through the route and compute the total travelling time
    int total_travelling_time = 0;
    for (int i = 0; i < route.id_sequence.size() - 1; i++) {
        int intervention_id = route.id_sequence[i];
        int next_intervention_id = route.id_sequence[i + 1];
        total_travelling_time += instance.time_matrix[intervention_id][next_intervention_id];
    }
    return total_travelling_time;
}

std::vector<std::pair<int, int>> imposed_routings_from_routes(const std::vector<Route>& routes, const IntegerSolution& integer_solution) {
    std::vector<std::pair<int, int>> imposed_routings;
    for (int r = 0; r < routes.size(); r++) {
        if (integer_solution.coefficients[r] == 0) continue;
        // Only if the route is actually used
        // Add every (vehicle_id, intervention_id) pair to the imposed routings
        // We skip the first and last node which are the depot
        for (int i = 1; i < routes[r].id_sequence.size() - 1; i++) {
            imposed_routings.push_back(std::make_pair(routes[r].vehicle_id, routes[r].id_sequence[i]));
        }
    }
    
    return imposed_routings;
}

std::vector<Route> keep_used_routes(const std::vector<Route>& routes, const IntegerSolution& integer_solution) {
    std::vector<Route> used_routes;
    for (int r = 0; r < routes.size(); r++) {
        if (integer_solution.coefficients[r] == 0) continue;
        used_routes.push_back(routes[r]);
    }
    return used_routes;
}


Route parse_route(const nlohmann::json & data, const Instance& instance) {
    using std::vector, std::string;
    int vehicle_id = data.at("vehicle_id");
    vector<int> sequence_julia = data.at("sequence");
    vector<int> start_times_data = data.at("start_times");
    // Build the real route object
    int n_nodes = instance.nodes.size();
    int n_vehicles = instance.vehicles.size();
    // Find the corresponding vehicle id in the instance
    // We have to find the vehicle whose technicians are the same as those in the json object
    int true_vehicle_id = -1;
    vector<string> technicians = data.at("technicians");
    for (int i = 0; i < n_vehicles; i++) {
        if (instance.vehicles[i].technicians == technicians) {
            true_vehicle_id = i;
            break;
        }
    }
    if (true_vehicle_id == -1) {
        throw std::invalid_argument("Vehicle not found in the instance");
    }
    vector<int> id_sequence = vector<int>();
    vector<int> is_in_route = vector<int>(n_nodes, 0);
    vector<vector<int>> route_edges = vector<vector<int>>(n_nodes, vector<int>(n_nodes, 0));
    // Initialize the sequence with the depot
    id_sequence.push_back(instance.vehicles[true_vehicle_id].depot);
    is_in_route[instance.vehicles[true_vehicle_id].depot] = 1;
    int duration = 0;

    vector<int> order = std::vector<int>(sequence_julia.size());
    std::iota(order.begin(), order.end(), 0);
    if (start_times_data.size() == sequence_julia.size()) {
        // Go through the sequence and the start times in order of increasing start time
        // (In the json object, the sequence is ordered by increasing node index)
        std::sort(order.begin(), order.end(), [&](int i, int j) { return start_times_data[i] < start_times_data[j]; });
    }

    // Now we go through the sequence in the order
    for (int i : order) {
        int node_id = sequence_julia[i] - 1;
        id_sequence.push_back(node_id);
        is_in_route[node_id] = 1;
        duration += instance.nodes.at(node_id).duration;
    }
    // Finally, also add the depot at the end
    id_sequence.push_back(instance.vehicles[true_vehicle_id].depot);
    // Build the route_edges matrix & compute the total cost
    double total_cost = instance.vehicles[true_vehicle_id].cost;
    for (int i = 0; i < id_sequence.size() - 1; i++) {
        total_cost += instance.distance_matrix[id_sequence[i]][id_sequence[i + 1]] * instance.cost_per_km;
        route_edges[id_sequence[i]][id_sequence[i + 1]] = 1;
    }

    return Route{
        true_vehicle_id,
        total_cost,
        0,
        duration,
        id_sequence,
        is_in_route,
        route_edges
    };
}


std::vector<Route> parse_routes_from_file(const std::string& filename, const Instance& instance) {
    using std::vector;
    using nlohmann::json;
    using std::cout, std::endl;

    // Read the file
    std::ifstream f(filename);
    json data = json::parse(f);
    f.close();
    // Print the meta-data
    cout << "-----------------------------------" << endl;
    cout << "Reading routes from file " << filename << endl;
    // cout << "Number of routes: " << data.at("nb_vehicles") << endl;
    // cout << "Expected objective value: " << data.at("objective") << endl;
    // cout << "Total intervention duration: " << data.at("duration") << endl;
    // cout << "Number of covered interventions: " << data.at("nb_interventions") << endl;
    // We now parse the routes themselves
    vector<Route> routes = vector<Route>();
    vector<json> routes_data = data.at("routes");
    for (const json& route_data : routes_data) {
        Route route = parse_route(route_data, instance);
        routes.push_back(route);
    }       

    return routes;
}


void print_route_reduced(const Route& route, const Instance& instance) {
    if (route.id_sequence.size() <= 2) {
        return;
    }
    using std::cout, std::endl;
    cout << "Vehicle " << route.vehicle_id;
    cout << " - Total cost : " << route.total_cost << " - Reduced cost : " << route.reduced_cost;
    cout << " - Total duration : " << route.total_duration << " - Total distance : " << count_route_kilometres(route, instance) << endl;
    cout << "Sequence : ";
    for (int i = 0; i < route.id_sequence.size(); i++) {
        cout << route.id_sequence[i] << " ";
    }
    cout << endl;
}

