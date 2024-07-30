// We will define in this file the class Solution that will be used to store the solution of the problem.

#pragma once
#include <vector>
#include <map>
#include <string>
#include <set>
#include "instance.h"
#include "../pathwyse/core/data/path.h"


// Structure to represent a solution of the master problem
struct MasterSolution {
    // Coefficients of the variables in the master problem
    std::vector<double> coefficients;
    // Dual values associated with the interventions
    std::vector<double> alphas;
    // Dual values associated with the vehicles
    std::vector<double> betas;
    // Objective value of the master problem
    double objective_value;
    // Empty constructor
    MasterSolution(){}
    // Constructor
    MasterSolution(std::vector<double> coefficients, std::vector<double> alphas, std::vector<double> betas, double objective_value){
        this->coefficients = coefficients;
        this->alphas = alphas;
        this->betas = betas;
        this->objective_value = objective_value;
    }
};

bool operator==(const MasterSolution& lhs, const MasterSolution& rhs);

// Solution to the integer version of the problem
struct IntegerSolution {
    std::vector<int> coefficients;
    double objective_value;
    // Empty constructor
    IntegerSolution(){}
    // Constructor
    IntegerSolution(std::vector<int> coefficients, double objective_value){
        this->coefficients = coefficients;
        this->objective_value = objective_value;
    }
};

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
    // Empty route constructor
    Route(int vehicle_id, int number_of_nodes){
        this->vehicle_id = vehicle_id;
        this->is_in_route = std::vector<int>(number_of_nodes, 0);
        this->start_times = std::vector<double>(number_of_nodes, 0);
        this->id_sequence = std::vector<int>();
    }
    // Constructor
    Route(int vehicle_id, double total_cost, double reduced_cost, double total_duration, double total_travelling_time, double total_waiting_time,
            std::vector<int> id_sequence, std::vector<int> is_in_route, std::vector<double> start_times){
        this->total_cost = total_cost;
        this->reduced_cost = reduced_cost;
        this->total_duration = total_duration;
        this->total_travelling_time = total_travelling_time;
        this->total_waiting_time = total_waiting_time;
        this->vehicle_id = vehicle_id;
        this->is_in_route = is_in_route;
        this->start_times = start_times;
        this->id_sequence = id_sequence;
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
