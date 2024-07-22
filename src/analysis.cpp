#include "analysis.h"
#include <map>
#include <string>
#include <set>
#include <algorithm>

using std::vector, std::map, std::string, std::set;
using std::find;
using std::cout, std::endl;

int count_covered_interventions(const IntegerSolution& solution, const vector<Route>& routes, const Instance& instance) {
    int nb_routes = routes.size();
    int nb_interventions = instance.number_interventions;
    vector<int> is_covered(nb_interventions, 0);

    // Go through all interventions
    for (int i = 0; i < nb_interventions; i++) {
        // Go through all routes
        for (int j = 0; j < nb_routes; j++) {
            // If the intervention is covered more than once, there is a problem
            if (solution.coefficients[j] > 1 && routes[j].is_in_route[i] == 1) {
                cout << "Intervention " << i << " is covered more than once" << endl;
                return -1;
            }
            // If the intervention is covered by the route, mark it as covered
            if (routes[j].is_in_route[i] == 1) {
                is_covered[i] = 1;
            }
        }
    }

    // Count the number of covered interventions
    int count = 0;
    for (int i = 0; i < nb_interventions; i++) {
        count += is_covered[i];
    }

    return count;
}

int count_used_vehicles(const IntegerSolution& solution, const vector<Route>& routes, const Instance& instance) {
    int nb_routes = routes.size();
    int nb_vehicles = instance.vehicles.size();
    vector<int> is_used(nb_vehicles, 0);

    // Go through all routes
    for (int r = 0; r < nb_routes; r++) {
        // Check that each vehicle is used at most once
        if (solution.coefficients[r] > 0 && is_used[routes[r].vehicle_id] > 0) {
            cout << "Vehicle " << routes[r].vehicle_id << " is used more than once" << endl;
            return -1;
        }
        // If the route is used, mark the vehicle as used
        if (solution.coefficients[r] > 0) {
            is_used[routes[r].vehicle_id] = 1;
        }
    }

    // Count the number of used vehicles
    int count = 0;
    for (int v = 0; v < nb_vehicles; v++) {
        count += is_used[v];
    }

    return count;
}


// Check that a route is feasible :
// - The route starts and ends at the depot
// - The route respects the time windows of the interventions
// - The route respects the capacities of the vehicles
bool is_route_feasible(const Route& route, const Instance& instance) {
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


double time_spent_travelling(const IntegerSolution& solution, const vector<Route>& routes, const Instance& instance) {
    double total_time = 0;
    for (int i = 0; i < routes.size(); i++) {
        if (solution.coefficients[i] > 0) {
            total_time += routes[i].total_travelling_time;
        }
    }
    return total_time;
}

double time_spent_working(const IntegerSolution& solution, const vector<Route>& routes, const Instance& instance) {
    double total_time = 0;
    for (int i = 0; i < routes.size(); i++) {
        if (solution.coefficients[i] > 0) {
            total_time += routes[i].total_duration;
        }
    }
    return total_time;
}

double time_spent_waiting(const IntegerSolution& solution, const vector<Route>& routes, const Instance& instance) {
    double total_time = 0;
    for (int i = 0; i < routes.size(); i++) {
        if (solution.coefficients[i] > 0) {
            total_time += routes[i].total_waiting_time;
        }
    }
    return total_time;
}


int count_coverable_interventions(const IntegerSolution& solution, const vector<Route>& routes, const Instance& instance) {
    int nb_routes = routes.size();
    int nb_interventions = instance.number_interventions;
    set<int> coverable_inters;
    // Get the indexes of the vehicles used in the solution
    vector<int> used_vehicles;
    for (int i = 0; i < nb_routes; i++) {
        if (solution.coefficients[i] > 0) {
            used_vehicles.push_back(routes[i].vehicle_id);
        }
    }

    // Go through all interventions
    for (int i = 0; i < nb_interventions; i++) {
        // Check if the intervention can be performed by the vehicles used in the solution
        const Node& intervention = instance.nodes[i];
        bool can_be_covered = false;
        for (int vehicle_id : used_vehicles) {
            const Vehicle& vehicle = instance.vehicles[vehicle_id];
            if (find(vehicle.interventions.begin(), vehicle.interventions.end(), i) != vehicle.interventions.end()) {
                can_be_covered = true;
                break;
            }
        }
        if (can_be_covered) {
            coverable_inters.insert(i);
        }
    }
    return coverable_inters.size();
}


int count_routes_with_duplicates(const vector<Route>& routes) {
    int nb_routes = routes.size();
    vector<int8_t> has_duplicate(nb_routes, 0);

    for (int i = 0; i < nb_routes; i++) {
        for (int j = i + 1; j < nb_routes; j++) {
            if (routes[i] == routes[j]) {
                has_duplicate[i] = 1;
                has_duplicate[j] = 1;
            }
        }
    }

    int count = 0;
    for (int i = 0; i < nb_routes; i++) {
        count += has_duplicate[i];
    }
    return count;
}

int count_used_routes_with_duplicates(const IntegerSolution& solution, const vector<Route>& routes) {
    int nb_routes = routes.size();
    vector<int8_t> is_used_and_has_duplicate(nb_routes, 0);
    vector<int> number_of_duplicates(nb_routes, 0);
    for (int i = 0; i < nb_routes; i++) {
        for (int j = i + 1; j < nb_routes; j++) {
            if (routes[i] == routes[j]) {
                number_of_duplicates[i]++;
                number_of_duplicates[j]++;
                if (solution.coefficients[i] > 0 ) {
                    is_used_and_has_duplicate[i] = 1;
                } else if (solution.coefficients[j] > 0) {
                    is_used_and_has_duplicate[j] = 1;
                }
            }
        }
    }

    int count = 0;
    for (int i = 0; i < nb_routes; i++) {
        count += is_used_and_has_duplicate[i];
    }
    return count;
}