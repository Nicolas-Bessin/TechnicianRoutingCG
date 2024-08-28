#pragma once

#include "instance.h"
#include "master.h"

#include <vector>
#include <map>
#include <string>
#include <set>


// Class Route : represents a sequence of nodes, starting at the warehouse, going through interventions and ending at the warehouse
struct Route {
    // Index of the vehicle that will perform the route
    int vehicle_id;
    // Total cost of the route
    double total_cost;
    // Reduced cost of the route at the time of creation
    double reduced_cost;
    // Total duration of the interventions along the route
    int total_duration;
    // Sequence of nodes in the route (not strictly necessary for the problem but useful for analysis)
    std::vector<int> id_sequence;
    // List of nodes in the route (1 if the node is in the route, 0 otherwise)
    std::vector<int> is_in_route;
    // Matrix of the travel path
    std::vector<std::vector<int>> route_edges;
    // Empty constructor
    Route(int n_nodes) : is_in_route(n_nodes), route_edges(n_nodes, std::vector<int>(n_nodes, 0)) {}
    // Constructor with all the attributes
    Route(
        int vehicle_id,
        double total_cost,
        double reduced_cost,
        int total_duration,
        std::vector<int> id_sequence,
        std::vector<int> is_in_route,
        std::vector<std::vector<int>> route_edges
    ) : 
        vehicle_id(vehicle_id),
        total_cost(total_cost),
        reduced_cost(reduced_cost),
        total_duration(total_duration),
        id_sequence(id_sequence),
        is_in_route(is_in_route),
        route_edges(route_edges) {}

};


// Checks if two routes are equal
// That is, if they have :
// - the same vehicle_id
// - the same sequence vector
// - the same start times
bool operator==(const Route& lhs, const Route& rhs);

// Comutes the length of a route
double count_route_kilometres(const Route& route, const Instance& instance);

// Computes the reduced cost of a route given the dual values of the constraints
double compute_reduced_cost(const Route& route, const std::vector<double>& alphas, double beta, const Instance& instance);

// Checks if a route is feasible
bool is_route_feasible(const Route& route, const Instance& instance);

// Returns a vector of the start times of the nodes along the route's sequence
std::vector<int> compute_start_times(const Route& route, const Instance& instance);

// Computes the total waiting time along a route (time not spent travelling nor performing interventions)
int compute_total_waiting_time(const Route& route, const Instance& instance);

// Computes the total travelling time along a route
int compute_total_travelling_time(const Route& route, const Instance& instance);


// Build a vector of pairs (vehicle_id, intervention_id) from the routes in an IntegerSolution
std::vector<std::pair<int, int>> imposed_routings_from_routes(const std::vector<Route>& routes, const IntegerSolution& integer_solution);

// Builds a new vector of routes containing only the routes that are used in the integer solution
std::vector<Route> keep_used_routes(const std::vector<Route>& routes, const IntegerSolution& integer_solution);

// Parse a vector of routes from a file
std::vector<Route> parse_routes_from_file(const std::string& filename, const Instance& instance);


