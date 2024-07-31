#pragma once

#include "instance.h"
#include "solution.h"

#include <vector>

#define NO_ROUTING 0
#define WARM_START 1
#define IMPOSE_ROUTING 2


/*
    Templated struct to store the solution of the compact model
*/
template <typename T>
struct CompactSolution{
    double objective_value;
    std::vector<std::vector<std::vector<T>>> x;
    std::vector<T> y;
    std::vector<T> u;

    // Default constructor
    CompactSolution() : objective_value(0.0), x(), y(), u() {}

    // Constructor with sizes
    CompactSolution(int n_nodes, int n_interventions, int n_vehicles) : objective_value(0.0), x(n_nodes, std::vector<std::vector<T>>(n_nodes, std::vector<T>(n_vehicles))), y(n_vehicles), u(n_interventions) {}
};

/*
    Solves the compact model for the problem
    @param instance: the instance of the problem
    @param time_limit: the time limit for the solver
    @param routes : a vector of routes to help the solver (default empty)
    @param mode : the mode of the solver help : NO_ROUTING, WARM_START, IMPOSE_ROUTING

    NO_ROUTING : no additional information is given to the solver   

    WARM_START : the routes are use to warm start the variables   

    IMPOSE_ROUTING : the routes are used to impose the routing of the vehicles  
    
    @return a CompactSolution struct with the values of the variables
*/
CompactSolution<int> compact_solver(const Instance& instance, int time_limit = 60, std::vector<Route> routes = std::vector<Route>(), int mode = NO_ROUTING);


std::vector<Route> compact_solution_to_routes(const Instance& instance, const CompactSolution<int>& compact_solution);