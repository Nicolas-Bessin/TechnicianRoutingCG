#include "analysis.h"

#include "instance/constants.h"

#include <iostream>
#include <map>
#include <string>
#include <set>
#include <algorithm>
#include <assert.h>
#include <iomanip>

using std::vector, std::map, std::string, std::set, std::pair;
using std::find;


int count_zeros(const vector<double>& vec) {
    int count = 0;
    for (int i = 0; i < vec.size(); i++) {
        if (vec[i] < 0.0001) {
            count++;
        }
    }
    return count;
}

double count_covered_interventions(const MasterSolution& solution, const vector<Route>& routes, const Instance& instance) {
    double count = 0;
    for (int i = 0; i < instance.number_interventions; i++) {
        for (int r = 0; r < routes.size(); r++) {
            count += solution.coefficients[r] * routes[r].is_in_route[i];
        }
    }
    return count;
}


// Returns a vector of size n_interventions, with a 1 at the index of each intervention that is covered by the solution
std::vector<int> covered_interventions(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance, bool details) {
    using std::cout, std::endl;
    
    int nb_routes = routes.size();
    int nb_interventions = instance.number_interventions;
    vector<int> is_covered(nb_interventions, 0);

    // Go through all interventions
    for (int i = 0; i < nb_interventions; i++) {
        // Go through all routes
        for (int r = 0; r < nb_routes; r++) {
            // If the intervention is covered more than once, there is a problem
            const Route& route = routes[r];
            if (solution.coefficients[r] > 0 && is_covered[i] > 0 && route.is_in_route[i] > 0) {
                if (details) cout << "Intervention " << i << " is covered more than once" << endl;
                is_covered[i] += 1;
            }
            // If the intervention is covered, mark it as covered
            else if (solution.coefficients[r] > 0 && route.is_in_route[i] > 0) {
                is_covered[i] = 1;
            }
        }
    }

    return is_covered;
}


int count_covered_interventions(const IntegerSolution& solution, const vector<Route>& routes, const Instance& instance) {
    using std::cout, std::endl;
    
    int nb_routes = routes.size();
    int nb_interventions = instance.number_interventions;
    vector<int> is_covered = covered_interventions(solution, routes, instance);

    // Count the number of covered interventions
    int count = 0;
    for (int i = 0; i < nb_interventions; i++) {
        count += is_covered[i];
    }
    return count;
}

vector<int> used_vehicles(const IntegerSolution& solution, const vector<Route>& routes, const Instance& instance) {
    using std::cout, std::endl;

    int nb_routes = routes.size();
    int nb_vehicles = instance.vehicles.size();
    vector<int> is_used(nb_vehicles, 0);

    // Go through all routes
    for (int r = 0; r < nb_routes; r++) {
        // If the route is used, mark the vehicle as used
        if (solution.coefficients[r] > 0) {
            is_used[routes[r].vehicle_id] = 1;
        }
    }

    return is_used;
}


double relaxed_solution_cost(const MasterSolution& solution, const vector<Route>& routes) {
    double total_cost = 0;
    for (int i = 0; i < routes.size(); i++) {
        total_cost += solution.coefficients[i] * (routes[i].total_cost);
    }
    return total_cost;
}

double fixed_cost(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance) {
    double total_cost = 0;
    for (int i = 0; i < routes.size(); i++) {
        if (solution.coefficients[i] > 0) {
            total_cost += instance.vehicles[routes[i].vehicle_id].cost;
        }
    }
    return total_cost;
}

int count_used_vehicles(const IntegerSolution& solution, const vector<Route>& routes, const Instance& instance) {
    using std::cout, std::endl;

    int nb_routes = routes.size();
    int nb_vehicles = instance.vehicles.size();
    vector<int> is_used = used_vehicles(solution, routes, instance);

    // Count the number of used vehicles
    int count = 0;
    for (int v = 0; v < nb_vehicles; v++) {
        count += is_used[v];
    }

    return count;
}

pair<double, int> count_used_vehicles(const MasterSolution& solution, const vector<Route>& routes, const Instance& instance) {
    using std::cout, std::endl;

    int nb_routes = routes.size();
    int nb_vehicles = instance.vehicles.size();
    double used_vehicles = 0;
    vector<int> is_used = vector<int>(nb_vehicles, 0);
    for (int r = 0; r < nb_routes; r++) {
        used_vehicles += solution.coefficients[r];
        if (solution.coefficients[r] > 0) {
            is_used[routes[r].vehicle_id] = 1;
        }
    }

    int unique_used_vehicles = 0;
    for (int e : is_used) {
        unique_used_vehicles += e;
    }

    return std::make_pair(used_vehicles, unique_used_vehicles);
}

void print_used_vehicles(const IntegerSolution& solution, const vector<Route>& routes, const Instance& instance) {
    using std::cout, std::endl;

    vector<int> is_used = used_vehicles(solution, routes, instance);
    int nb_vehicles = instance.vehicles.size();

    // Print the vehicles that are used
    cout << "Used vehicles: ";
    for (int i = 0; i < nb_vehicles; i++) {
        if (is_used[i] > 0) {
            cout << i << ", ";
        }
    }
    cout << endl;
}

void check_feasibility(const IntegerSolution& solution, const vector<Route>& routes, const Instance& instance) {
    using std::cout, std::endl;
    bool all_feasible = true;
    for (int i = 0; i < routes.size(); i++){
        const Route& route = routes.at(i);
        if (solution.coefficients[i] > 0 && !is_route_feasible(route, instance)){
            cout << "Route " << i << " is not feasible" << endl;
            all_feasible = false;
        }
    }
    if (all_feasible){
        cout << "All routes are feasible" << endl;
    }
}

double time_spent_travelling(const IntegerSolution& solution, const vector<Route>& routes, const Instance& instance) {
    double total_time = 0;
    for (int i = 0; i < routes.size(); i++) {
        if (solution.coefficients[i] > 0) {
            total_time += compute_total_travelling_time(routes[i], instance);
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
            total_time += compute_total_waiting_time(routes[i], instance);
        }
    }
    return total_time;
}

int shortest_time_route(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance) {
    int shortest_time = END_DAY; // Set to the maximum possible time
    for (int r = 0; r < routes.size(); r++) {
        if (solution.coefficients[r] > 0) {
            int route_time = routes[r].total_duration + compute_total_waiting_time(routes[r], instance) + compute_total_travelling_time(routes[r], instance);
            shortest_time = std::min(shortest_time, route_time);
        }
    }
    return shortest_time;
}

int longest_time_route(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance) {
    int longest_time = 0;
    for (int r = 0; r < routes.size(); r++) {
        if (solution.coefficients[r] > 0) {
            int route_time = routes[r].total_duration + compute_total_waiting_time(routes[r], instance) + compute_total_travelling_time(routes[r], instance);
            longest_time = std::max(longest_time, route_time);
        }
    }
    return longest_time;
}

void print_used_route_durations(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance) {
    using std::cout, std::endl;
    cout << "Route durations: ";
    for (int r = 0; r < routes.size(); r++) {
        if (solution.coefficients[r] > 0) {
            int route_time = routes[r].total_duration + compute_total_waiting_time(routes[r], instance) + compute_total_travelling_time(routes[r], instance);
            cout << std::setw(7) << "v" << routes[r].vehicle_id << " : " << route_time << ", ";
        }
    }
    cout << endl;
}

// Prints the objective coefficients of the used routes
void print_used_route_obj_coeffs(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance) {
    using std::cout, std::endl;
    cout << "Route objective coefficients: ";
    for (int r = 0; r < routes.size(); r++) {
        if (solution.coefficients[r] > 0) {
            double coef = instance.M * routes[r].total_duration;
            coef -= instance.cost_per_km * count_route_kilometres(routes[r], instance);
            coef -= instance.vehicles[routes[r].vehicle_id].cost;
            cout << std::setw(2) << std::setprecision(1) << std::fixed << "v" << routes[r].vehicle_id << " : " << coef << ", ";
        }
    }
    cout << endl;
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


/*
    Count the number of routes that have a duplicate anywhere in the list of routes.
    Thus, if there is a pair of equal routes, both will be counted.
*/
int count_routes_with_duplicates(const vector<Route>& routes) {
    int nb_routes = routes.size();
    vector<int8_t> has_duplicate(nb_routes, 0);

    for (int i = 0; i < nb_routes; i++) {
        for (int j = i + 1; j < nb_routes; j++) {
            if (routes[i] == routes[j]) {
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


double count_kilometres_travelled(const IntegerSolution& solution, const vector<Route>& routes, const Instance& instance) {
    double total_distance = 0;
    for (int i = 0; i < routes.size(); i++) {
        if (solution.coefficients[i] > 0) {
            total_distance += count_route_kilometres(routes[i], instance);
        }
    }
    return total_distance;
}


/*
    Returns the accumulated reduced cost until this node in the sequence of nodes for a route
*/
vector<double> accumulated_reduced_cost(const Route& route, const MasterSolution& solution, const Instance& instance) {
    const Vehicle& vehicle = instance.vehicles[route.vehicle_id];
    // Initialize the accumulated cost vector as an empty vector
    vector<double> accumulated_costs = {};
    // Take into account the vehicle's fixed cost and the beta value
    double current_cost = vehicle.cost + solution.dual_solution.betas[route.vehicle_id];

    // Then go through the sequence of nodes and accumulate the reduced cost
    for (int i = 0; i < route.id_sequence.size(); i++) {
        int true_i = route.id_sequence[i];
        if (true_i != vehicle.depot) {
            current_cost -= solution.dual_solution.alphas[true_i];
        }
        // We count the accumulated reduced cost until this node - so we add it after adding the alpha from the current node
        accumulated_costs.push_back(current_cost);
        // If there is a next node, we add the corresponding travel cost
        if (i < route.id_sequence.size() - 1) {
            int true_j = route.id_sequence[i + 1];
            current_cost += instance.cost_per_km * instance.distance_matrix[true_i][true_j];
            // If applicable, also add the duals from the cuts of the BP tree
            std::tuple<int, int, int> edge = {true_i, true_j, route.vehicle_id};
            if (solution.dual_solution.lower_bound_duals.contains(edge)) {
                current_cost -= solution.dual_solution.lower_bound_duals.at(edge);
            }
            if (solution.dual_solution.upper_bound_duals.contains(edge)) {
                current_cost += solution.dual_solution.upper_bound_duals.at(edge);
            }
        }
    }
    return accumulated_costs;
}


void print_route(const Route & route, const Instance & instance, const MasterSolution& solution) {
    using std::cout, std::endl;
    using std::setprecision, std::fixed, std::setw;

    // If the route is empty, print a message and return
    if (route.id_sequence.size() == 0) {
        cout << "Empty route" << endl;
        return;
    }

    // Print the vehicle id
    cout << "Vehicle id: " << route.vehicle_id << " - Tour length: " << route.id_sequence.size() << endl;
    cout << "Technicians in the vehicle: ";
    for (string tech_id : instance.vehicles[route.vehicle_id].technicians) {
        cout << tech_id << ", ";
    }
    cout << endl;
    
    // Print the tvehicle cost, travelling cost and total cost
    double travel_distance = count_route_kilometres(route, instance);
    cout << setprecision(1) << fixed;
    cout << "Vehicle cost: " << instance.vehicles[route.vehicle_id].cost;
    cout << " - Travelling cost: " << travel_distance * instance.cost_per_km;
    cout << " - Total distance: " << travel_distance;
    cout << " - Total cost: " << route.total_cost << endl;
    // Print the total duration, travelling time and waiting time
    cout << "Total duration: " << route.total_duration;
    cout << " - Total travelling time: " << compute_total_travelling_time(route, instance);
    cout << " - Total waiting time: " << compute_total_waiting_time(route, instance) << endl;
    // Print the objective coefficient of the route
    double coef = instance.M * route.total_duration - route.total_cost;
    cout << "Reduced cost: " << route.reduced_cost;
    cout << " - Objective coefficient: " << coef << endl;
    // Set the column width in the tab
    int col_width = 4;
    if (solution.is_feasible) {
        col_width = 6;
    }
    // Print the sequence of nodes
    cout << "Sequence:     |";
    for (int i = 0; i < route.id_sequence.size(); i++) {
        cout << setw(col_width) << route.id_sequence[i] << ", ";
    }
    cout << endl;
    // cout << "Sequence (node_id): ";
    // for (int i = 0; i < route.id_sequence.size(); i++) {
    //     cout << instance.nodes[route.id_sequence[i]].node_id << ", ";
    // }
    cout << "--------------+--------------------------------------------------------------------------" << endl;
    // Print the start of time window, real start time, Duration, travel to next and end of time window
    cout << "Window start: |";
    for (int i = 0; i < route.id_sequence.size(); i++) {
        cout << setw(col_width) << instance.nodes[route.id_sequence[i]].start_window << ", ";
    }
    cout << endl;
    cout << "Start time:   |";
    vector<int> start_times = compute_start_times(route, instance);
    for (int i = 0; i < route.id_sequence.size(); i++) {
        cout << setw(col_width) << start_times[i] << ", ";
    }
    cout << endl;
    cout << "Duration:     |";
    for (int i = 0; i < route.id_sequence.size(); i++) {
        cout << setw(col_width) << instance.nodes[route.id_sequence[i]].duration << ", ";
    }
    cout << endl;
    cout << "Travel time:  |";
    for (int i = 0; i < route.id_sequence.size() - 1; i++) {
        cout << setw(col_width) << instance.time_matrix[route.id_sequence[i]][route.id_sequence[i + 1]] << ", ";
    }
    cout << setw(col_width) << "   -, " << endl;
    cout << "Travel length:|";
    for (int i = 0; i < route.id_sequence.size() - 1; i++) {
        cout << setw(col_width) << instance.distance_matrix[route.id_sequence[i]][route.id_sequence[i + 1]] << ", ";
    }
    cout << setw(col_width) << "   -, " << endl;
    // Only print the RC information if a MasterSolution is provided
    if (solution.is_feasible) {
        cout << "Accu RC       |";
        vector<double> accu_rc = accumulated_reduced_cost(route, solution, instance);
        for (int i = 0; i < accu_rc.size(); i++) {
            cout << setw(col_width) << accu_rc[i] << ", ";
        }
        cout << endl;
    }
    cout << "Window end:   |";
    for (int i = 0; i < route.id_sequence.size(); i++) {
        cout << setw(col_width) << instance.nodes[route.id_sequence[i]].end_window << ", ";
    }
    cout << endl;



}


void print_used_routes(const IntegerSolution& solution, const vector<Route>& routes, const Instance& instance) {
    using std::cout, std::endl;
    for (int i = 0; i < routes.size(); i++) {
        if (solution.coefficients[i] > 0) {
            print_route(routes[i], instance);
            cout << "----------------" << endl;
        }
    }
}




void print_non_covered_interventions(const IntegerSolution& solution, const vector<Route>& routes, const Instance& instance, bool details) {
    using std::cout, std::endl;
    int nb_interventions = instance.number_interventions;
    vector<int> is_covered = covered_interventions(solution, routes, instance);
    if (!details) {
        // Print the interventions that are not covered
        cout << "Non-realised interventions: ";
        for (int i = 0; i < nb_interventions; i++) {
            if (is_covered[i] == 0) {
                cout << i << ", ";
            }
        }
        cout << endl;
        return;
    }
    // Print the interventions that are not covered with details
    cout << "Non-realised interventions: " << endl;
    for (int i = 0; i < nb_interventions; i++) {
        if (is_covered[i] == 0) {
            cout << "Intervention " << i << endl;
            cout << "Start window: " << instance.nodes[i].start_window << " ";
            cout << "End window: " << instance.nodes[i].end_window << " ";
            cout << "Duration: " << instance.nodes[i].duration << endl;
            cout << "Consumption: ";
            for (const auto& [label , value] : instance.nodes[i].quantities) {
                cout << label << ": " << value << " ";
            }
            cout << endl;
            cout << "----------------" << endl;
        }
    }
}

void print_vehicles_non_covered(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance) {
    using std::cout, std::endl;
    
    int nb_interventions = instance.number_interventions;
    int nb_vehicles = instance.vehicles.size();

    vector<int> is_covered = covered_interventions(solution, routes, instance);
    // Can a vehicle cover the non covered interventions ?
    vector<int> can_cover(nb_vehicles, 0);
    vector<int> is_used = used_vehicles(solution, routes, instance);

    // For each vehicle, check if it can cover at least N of the non covered interventions
    for (int v = 0; v < nb_vehicles; v++) {
        const Vehicle& vehicle = instance.vehicles[v];
        for (int i = 0; i < nb_interventions; i++) {
            if (is_covered[i] == 1) continue;
            const Node& intervention = instance.nodes[i];
            // Check if the vehicle can cover the intervention
            if (find(vehicle.interventions.begin(), vehicle.interventions.end(), i) != vehicle.interventions.end()) {
                can_cover[v]++;
            }

        }
    }

    // Print the number of non-covered interventions that can be covered by each vehicle
    cout << "Number of non covered interventions that can be covered by each non used  vehicle: " << endl;
    for (int v = 0; v < nb_vehicles; v++) {
        if (is_used[v] == 0) { 
            cout << "v" << v << " : " << can_cover[v] <<" - ";
        }
    }
    cout << endl;

}


void full_analysis(const IntegerSolution& integer_solution, const vector<Route>& routes, const Instance& instance, bool details) {
    using std::cout, std::endl;
    // Solution analysis

    cout << "-----------------------------------" << endl;
    // //print_used_routes(integer_solution, routes, instance); 
    // print_non_covered_interventions(integer_solution, routes, instance, false);
    // cout << "-----------------------------------" << endl;
    // print_used_vehicles(integer_solution, routes, instance);
    // print_vehicles_non_covered(integer_solution, routes, instance);
    // cout << "-----------------------------------" << endl;

    cout << "Number of covered interventions : " << count_covered_interventions(integer_solution, routes, instance);
    cout << " / " << instance.number_interventions << endl;

    cout << "Number of used vehicles : " << count_used_vehicles(integer_solution, routes, instance);
    cout << " / " << instance.vehicles.size() << endl;

    // Compute the sum of the vehicles fixed costs
    int fixed_costs = 0;
    vector<int> is_vehicle_used = used_vehicles(integer_solution, routes, instance);
    for (int i = 0; i < is_vehicle_used.size(); i++) {
        if (is_vehicle_used[i] > 0) {
            fixed_costs += instance.vehicles[i].cost;
        }
    }

    //cout << "Number of interventions that could be covered : " << count_coverable_interventions(integer_solution, routes, instance) << endl;

    // Check that all used routes are feasible
    bool all_feasible = true;
    auto covered = covered_interventions(integer_solution, routes, instance, true);
    for (int i : covered ) {
        if (i > 1) {
            all_feasible = false;
        }
    }
    for (int i = 0; i < routes.size(); i++){
        const Route& route = routes.at(i);
        if (integer_solution.coefficients[i] > 0 && !is_route_feasible(route, instance)){
            cout << "Route " << i << " is not feasible" << endl;
            print_route(route, instance);
            cout << "-----" << endl;
            all_feasible = false;
        }
    }
    if (all_feasible){
        cout << "All routes are feasible" << endl;
    }
    int working_time = time_spent_working(integer_solution, routes, instance);
    int travelling_time = time_spent_travelling(integer_solution, routes, instance);
    int waiting_time = time_spent_waiting(integer_solution, routes, instance);
    int total_time = working_time + travelling_time + waiting_time;
    cout << "Number of routes with duplicates : " << count_routes_with_duplicates(routes) << " / " << routes.size() << endl;
    if (details) {
        cout << "Number of used routes with duplicates : " << count_used_routes_with_duplicates(integer_solution, routes) << " / " << routes.size() << endl;
    }
    cout << "Number of route kilometres : " << count_kilometres_travelled(integer_solution, routes, instance) << " km";
    cout << " - Total fixed costs : " << fixed_costs << endl;
    if (details) {
        cout << "Time spent travelling : " << travelling_time << " minutes";
        cout << "- Time spent working : " << working_time << " minutes";
        cout << " - Time spent waiting : " << waiting_time << " minutes" << endl;
        int shortest_time = shortest_time_route(integer_solution, routes, instance);
        int longest_time = longest_time_route(integer_solution, routes, instance);
        cout << "Total time : " << total_time << " minutes - Average per route : " << total_time / count_used_vehicles(integer_solution, routes, instance) << " minutes" << endl;
        cout << "Shortest route time : " << shortest_time << " minutes - Longest route time : " << longest_time << " minutes" << endl;
        print_used_route_durations(integer_solution, routes, instance);
        print_used_route_obj_coeffs(integer_solution, routes, instance);
        print_non_covered_interventions(integer_solution, routes, instance, false);
        print_vehicles_non_covered(integer_solution, routes, instance);
    }

}