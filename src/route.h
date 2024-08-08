#pragma once

#include "instance.h"
#include "master.h"

#include <vector>
#include <map>
#include <string>
#include <set>


// Class Route : represents a sequence of nodes, starting at the warehouse, going through interventions and ending at the warehouse
struct Route {
    // Total cost of the route
    double total_cost;
    // Reduced cost of the route at the time of creation
    double reduced_cost;
    // Total duration of the interventions along the route
    double total_duration;
    // Total travelling time of the route
    double total_travelling_time;
    // Total waiting time of the route
    double total_waiting_time;
    // Index of the vehicle that will perform the route
    int vehicle_id;
    // Sequence of nodes in the route (not strictly necessary for the problem but useful for analysis)
    std::vector<int> id_sequence;
    // List of nodes in the route (1 if the node is in the route, 0 otherwise)
    std::vector<int> is_in_route;
    // Start time of these interventions
    std::vector<double> start_times;
    // Matrix of the travel path
    std::vector<std::vector<int>> route_edges;
    // Empty constructor
    Route(int n_nodes) {
        total_cost = 0;
        reduced_cost = 0;
        total_duration = 0;
        total_travelling_time = 0;
        total_waiting_time = 0;
        vehicle_id = -1;
        id_sequence = std::vector<int>();
        is_in_route = std::vector<int>(n_nodes, 0);
        start_times = std::vector<double>(n_nodes, 0);
        route_edges = std::vector<std::vector<int>>(n_nodes, std::vector<int>(n_nodes, 0));
    }

};


// Checks if two routes are equal
// That is, if they have :
// - the same vehicle_id
// - the same sequence vector
// - the same start times
bool operator==(const Route& lhs, const Route& rhs);


// To be able to define a set of routes:
bool operator<(const Route& lhs, const Route& rhs);


// Computes the reduced cost of a route given the dual values of the constraints
double compute_reduced_cost(const Route& route, const std::vector<double>& alphas, double beta, const Instance& instance);

// Checks if a route is feasible
bool is_route_feasible(const Route& route, const Instance& instance);


// Build a vector of pairs (vehicle_id, intervention_id) from the routes in an IntegerSolution
std::vector<std::pair<int, int>> imposed_routings_from_routes(const std::vector<Route>& routes, const IntegerSolution& integer_solution);

// Builds a new vector of routes containing only the routes that are used in the integer solution
std::vector<Route> keep_used_routes(const std::vector<Route>& routes, const IntegerSolution& integer_solution);
