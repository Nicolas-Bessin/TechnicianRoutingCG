#include "analysis.h"
#include "constants.h"
#include <map>
#include <string>
#include <set>
#include <algorithm>
#include <assert.h>
#include <iomanip>

using std::vector, std::map, std::string, std::set;
using std::find;


// Returns a vector of size n_interventions, with a 1 at the index of each intervention that is covered by the solution
std::vector<int> covered_interventions(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance) {
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
                cout << "Intervention " << i << " is covered more than once" << endl;
                is_covered[i] += 1;
            }
            // If the intervention is covered, mark it as covered
            if (solution.coefficients[r] > 0 && route.is_in_route[i] > 0) {
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


double count_route_kilometres(const Route& route, const Instance& instance) {
    double total_distance = 0;
    for (int i = 0; i < route.id_sequence.size() - 1; i++) {
        const Node& node1 = instance.nodes[route.id_sequence[i]];
        const Node& node2 = instance.nodes[route.id_sequence[i + 1]];
        total_distance += metric(node1, node2, instance.distance_matrix);
    }
    return total_distance;
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


double compute_integer_objective(const IntegerSolution& solution, const vector<Route>& routes, const Instance& instance) {
    double value = 0;
    assert (solution.coefficients.size() == routes.size());
    for (int r = 0; r < routes.size(); r++) {
        if (solution.coefficients[r] > 0) {
            double coef = instance.M * routes[r].total_duration;
            coef -= instance.cost_per_km * count_route_kilometres(routes[r], instance);
            coef -= instance.vehicles[routes[r].vehicle_id].cost;
            value += coef;
        }
    }
    return value;
}


void print_route(const Route & route, const Instance & instance) {
    using std::cout, std::endl;
    using std::setprecision, std::fixed;

    // Print the vehicle id
    cout << "Vehicle id: " << route.vehicle_id << " - Tour length: " << route.id_sequence.size() << endl;
    cout << "Technicians in the vehicle: ";
    for (string tech_id : instance.vehicles[route.vehicle_id].technicians) {
        cout << tech_id << ", ";
    }
    cout << endl;
    
    // Print the tvehicle cost, travelling cost and total cost
    double travel_distance = count_route_kilometres(route, instance);
    //cout << "Vehicle cost: " << instance.vehicles[route.vehicle_id].cost << " ";
    //cout << "Travelling cost: " << travel_distance * instance.cost_per_km << " ";
    //cout << "Total cost: " << route.total_cost << endl;
    // Print the kilometres along the route
    //cout << "Total distance: " << travel_distance << endl;
    // Print the total duration, travelling time and waiting time
    //cout << "Total duration: " << route.total_duration << " ";
    //cout << "Total travelling time: " << route.total_travelling_time << " ";
    //cout << "Total waiting time: " << route.total_waiting_time << endl;
    // Print the sequence of nodes
    cout << "Sequence: ";
    for (int i = 0; i < route.id_sequence.size(); i++) {
        cout << route.id_sequence[i] << ", ";
    }
    cout << endl;
    // cout << "Sequence (node_id): ";
    // for (int i = 0; i < route.id_sequence.size(); i++) {
    //     cout << instance.nodes[route.id_sequence[i]].node_id << ", ";
    // }
    cout << endl;
    // Print the start of time window, real start time, Duration, travel to next and end of time window
    cout << "Window start: ";
    for (int i = 0; i < route.id_sequence.size(); i++) {
        cout << setprecision(1) << instance.nodes[route.id_sequence[i]].start_window << ", ";
    }
    cout << endl;
    cout << "Start time:   ";
    for (int i = 0; i < route.id_sequence.size(); i++) {
        cout << setprecision(0) << route.start_times[route.id_sequence[i]] << ", ";
    }
    cout << endl;
    cout << "Duration:     ";
    for (int i = 0; i < route.id_sequence.size(); i++) {
        cout << setprecision(1) << instance.nodes[route.id_sequence[i]].duration << ", ";
    }
    cout << endl;
    cout << "Travel time:  ";
    for (int i = 0; i < route.id_sequence.size() - 1; i++) {
        cout << setprecision(1) << metric(instance.nodes[route.id_sequence[i]], instance.nodes[route.id_sequence[i + 1]], instance.time_matrix) << ", ";
    }
    cout << " -, " << endl;
    cout << "Window end:   ";
    for (int i = 0; i < route.id_sequence.size(); i++) {
        cout << setprecision(1) << instance.nodes[route.id_sequence[i]].end_window << ", ";
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




void print_non_covered_interventions(const IntegerSolution& solution, const vector<Route>& routes, const Instance& instance) {
    using std::cout, std::endl;
    int nb_interventions = instance.number_interventions;
    vector<int> is_covered = covered_interventions(solution, routes, instance);

    // Print the interventions that are not covered
    cout << "Non-realised interventions: ";
    for (int i = 0; i < nb_interventions; i++) {
        if (is_covered[i] == 0) {
            cout << i << ", ";
        }
    }
    cout << endl;
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
    cout << "Number of non covered interventions that can be covered by non used each vehicle: " << endl;
    for (int v = 0; v < nb_vehicles; v++) {
        if (is_used[v] == 0) {
            cout << "v" << v << " : " << can_cover[v] <<" - ";
        }
    }

}