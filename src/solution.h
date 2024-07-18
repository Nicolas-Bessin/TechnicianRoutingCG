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
    // Index of the vehicle that will perform the route
    int vehicle_id;
    // List of nodes in the route (1 if the intervention is in the route, 0 otherwise)
    std::vector<int> is_in_route;
    // Start time of these interventions
    std::vector<double> start_times;
    // Empty route constructor
    Route(int vehicle_id, int number_of_nodes){
        this->vehicle_id = vehicle_id;
        this->is_in_route = std::vector<int>(number_of_nodes, 0);
        this->start_times = std::vector<double>(number_of_nodes, 0);
    }
    // Constructor
    Route(double total_cost, double reduced_cost, double total_duration, int vehicle_id, std::vector<int> is_in_route, std::vector<double> start_times){
        this->total_cost = total_cost;
        this->reduced_cost = reduced_cost;
        this->total_duration = total_duration;
        this->vehicle_id = vehicle_id;
        this->is_in_route = is_in_route;
        this->start_times = start_times;
    }
};
