#pragma once

#include "instance/instance.h"
#include "master_problem/master.h"

#include <vector>
#include <map>
#include <string>
#include <set>


// Holds all the useful information about a route
struct Route {
    // Index of the vehicle that will perform the route
    int vehicle_id;
    // Total cost of the route
    double total_cost;
    // Reduced cost of the route at the time of creation
    double reduced_cost;
    // Total duration of the interventions along the route 
    int total_duration;
    // Sequence of the interventions along the route
    std::vector<int> id_sequence;
    // Vector of booleans indicating if an intervention is in the route
    std::vector<int> is_in_route;
    // Matrix of the travel path
    std::vector<std::vector<int>> route_edges;
};

// Creates an empty route in a graph with N vertices
Route EmptyRoute(const int N);


// Convert a partial path and its associated reduced cost to a Route object
// @param rc: the reduced cost of the path
// @param sequence: the sequence of the path in terms of in-vehicle indices
// @param instance: the instance of the problem
// @param vehicle: the vehicle that will perform the route
Route convert_sequence_to_route(double rc, const std::vector<int> & sequence, const Instance& instance, const Vehicle& vehicle);


// Checks if two routes are equal
// That is, if they have :
// - the same vehicle_id
// - the same sequence vector
// - the same start times
bool operator==(const Route& lhs, const Route& rhs);

// Comutes the length of a route
double count_route_kilometres(const Route& route, const Instance& instance);

// Comptues the total duration of interventions along a route
int count_route_duration(const Route& route, const Instance& instance);

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

// Print reduced information about a route (vehicle id, total cost, reduced cost, total duration and sequence)
void print_route_reduced(const Route& route, const Instance& instance);


