#include "route.h"

#include "master.h"
#include "constants.h"

#include <math.h>
#include <fstream>
#include <iostream>
#include <algorithm>

#include "../nlohmann/json.hpp"


Route::Route(
        int vehicle_id,
        double total_cost,
        double reduced_cost,
        int total_duration,
        int total_travelling_time,
        int total_waiting_time,
        std::vector<int> id_sequence,
        std::vector<int> is_in_route,
        std::vector<int> start_times
    ) : 
    vehicle_id(vehicle_id),
    total_cost(total_cost),
    reduced_cost(reduced_cost),
    total_duration(total_duration),
    total_travelling_time(total_travelling_time),
    total_waiting_time(total_waiting_time),
    id_sequence(id_sequence),
    is_in_route(is_in_route),
    start_times(start_times)
    {
        // Initialize the route_edges matrix
        int n_nodes = is_in_route.size();
        route_edges = std::vector<std::vector<int>>(n_nodes, std::vector<int>(n_nodes, 0));
        for (int i = 0; i < id_sequence.size() - 1; i++) {
            route_edges[id_sequence[i]][id_sequence[i + 1]] = 1;
        }
    }


using std::min;

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
        // Check that the start time is the current time 
        // (for the first node which is the depot, the start time will hold the arrival time at the end of the day)
        if (i > 0 && route.start_times[intervention_id] > current_time) {
            // This means we arrived too early - we thus wait until the time of the intervention
            current_time = route.start_times[intervention_id];
        }
        // If we are too late, the route is not feasible
        if (route.start_times[intervention_id] < current_time) {
            cout << "Start time of intervention " << intervention_id << " is not the time of arrival" << endl;
            cout << "Start time: " << route.start_times[intervention_id] << " Current time: " << current_time << endl;
            return false;
        }
        // Check that the time window is respected (with the way we built the solution, we can't arrive too early)
        if (current_time < intervention.start_window) {
            cout << "Intervention " << intervention_id << " starts too early";
            cout << " : " << current_time << " < " << intervention.start_window << endl;
            return false;
        }
        if (current_time + duration > intervention.end_window) {
            cout << std::setprecision(4) << "Intervention " << intervention_id << " ends too late : ";
            cout << current_time + duration << " > " << intervention.end_window << endl;
            return false;
        }
        // Check wether the lunch break is respected
        // If  we arrive at a time where we can't begin the intervention before the lunch break, we wait out the lunch break
        if (intervention.is_ambiguous && current_time < MID_DAY && current_time + duration > MID_DAY) {
            current_time = MID_DAY;
            // cout << "Intervention " << intervention_id << " ends after the lunch break : start = " << current_time << " end = " << current_time + duration << endl;
            // return false;
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
    vector<int> start_times = vector<int>(n_nodes, 0);
    vector<vector<int>> route_edges = vector<vector<int>>(n_nodes, vector<int>(n_nodes, 0));
    // Initialize the sequence with the depot
    id_sequence.push_back(instance.vehicles[true_vehicle_id].depot);
    is_in_route[instance.vehicles[true_vehicle_id].depot] = 1;
    start_times[instance.vehicles[true_vehicle_id].depot] = 0;
    int duration = 0;
    // Go through the sequence and the start times in order of increasing start time
    // (In the json object, the sequence is ordered by increasing node index)
    vector<int> order = std::vector<int>(sequence_julia.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int i, int j) { return start_times_data[i] < start_times_data[j]; });
    // Now we go through the sequence in the order of increasing start time
    for (int i : order) {
        int node_id = sequence_julia[i] - 1;
        id_sequence.push_back(node_id);
        is_in_route[node_id] = 1;
        start_times[node_id] = start_times_data[i];
        duration += instance.nodes[node_id].duration;
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
        -1.0,
        duration,
        -1,
        -1,
        id_sequence,
        is_in_route,
        start_times,
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
    cout << "Number of routes: " << data.at("nb_vehicles") << endl;
    cout << "Expected objective value: " << data.at("objective") << endl;
    cout << "Total intervention duration: " << data.at("duration") << endl;
    cout << "Number of covered interventions: " << data.at("nb_interventions") << endl;
    // We now parse the routes themselves
    vector<Route> routes = vector<Route>();
    vector<json> routes_data = data.at("routes");
    for (const json& route_data : routes_data) {
        Route route = parse_route(route_data, instance);
        routes.push_back(route);
    }       

    return routes;
}

