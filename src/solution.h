// We will define in this file the class Solution that will be used to store the solution of the problem.

#pragma once
#include <vector>
#include <map>
#include <string>
#include <set>
#include "instance.h"

using namespace std;


// Class Route : represents a sequence of nodes, starting at the warehouse, going through interventions and ending at the warehouse
class Route {
    public:
        int id;
        double reduced_cost;
        double total_cost;
        // Total duration of the interventions along the route
        double total_duration;
        // Total travel time along the route
        double travel_time;
        // Index of the vehicle that will perform the route
        int vehicle_id;
        // List of nodes in the route (1 if the intervention is in the route, 0 otherwise)
        vector<int> is_in_route;
        // Start time of these interventions
        vector<double> start_times;
        // Empty route constructor
        Route(int id, int vehicle_id, int number_of_nodes){
            this->id = id;
            this->vehicle_id = vehicle_id;
            this->is_in_route = vector<int>(number_of_nodes, 0);
            this->start_times = vector<double>(number_of_nodes, 0);
        }
        // Constructor
        Route(int id, double reduced_cost, double total_cost, double total_duration, double travel_time, int vehicle_id, vector<int> is_in_route, vector<double> start_times){
            this->id = id;
            this->reduced_cost = reduced_cost;
            this->total_cost = total_cost;
            this->total_duration = total_duration;
            this->travel_time = travel_time;
            this->vehicle_id = vehicle_id;
            this->is_in_route = is_in_route;
            this->start_times = start_times;
        }
        // Destructor
        ~Route(){}
};


//Class Solution : represents a solution to the problem
class Solution {
    public:
        // List of routes in the solution
        vector<Route*> routes;
        // Coefficients for these routes (value of the variables in the master problem)
        vector<double> coefficients;
        // Dual values associated with the interventions
        vector<double> alphas;
        // Dual values associated with the vehicles
        vector<double> betas;
        // Total cost of the solution
        double total_cost;
        // Total duration of the solution
        double total_duration;
        // Total travel time of the solution
        double total_travel_time;
        // Empty constructor
        Solution(){}
        // Constructor
        Solution(vector<Route*> routes, vector<double> coefficients, vector<double> alphas, vector<double> betas, double total_cost, double total_duration, double total_travel_time){
            this->routes = routes;
            this->coefficients = coefficients;
            this->alphas = alphas;
            this->betas = betas;
            this->total_cost = total_cost;
            this->total_duration = total_duration;
            this->total_travel_time = total_travel_time;
        }
        // Destructor
        ~Solution(){}
        
};

