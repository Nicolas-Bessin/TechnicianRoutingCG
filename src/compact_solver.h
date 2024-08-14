#pragma once

#include "instance.h"
#include "master.h"
#include "route.h"
#include "compact_solution.h"

#include <vector>

#define NO_ROUTING 0
#define WARM_START 1
#define IMPOSE_ROUTING 2


/*
    Solves the compact model for the problem
    @param instance: the instance of the problem
    @param time_limit: the time limit for the solver
    @param routes : a vector of routes to help the solver (default empty)
    @param mode : the mode of the solver help : NO_ROUTING, WARM_START, IMPOSE_ROUTING

    NO_ROUTING : no additional information is given to the solver   

    WARM_START : the routes are use to warm start the variables   

    IMPOSE_ROUTING : the routes are used to impose the routing of the vehicles  

    @param verbose : a boolean to print the output of the solver
    
    @return a CompactSolution struct with the values of the variables
*/
CompactSolution<int> compact_solver(const Instance& instance, int time_limit = 60, std::vector<Route> routes = std::vector<Route>(), int mode = NO_ROUTING, bool verbose = false);


/*
    Solves the compact model for the problem, with the x and y variables relaxed
    @param instance: the instance of the problem
    @param time_limit: the time limit for the solver
    @param verbose : a boolean to print the output of the solver
    
    @return a CompactSolution struct with the values of the variables
*/
CompactSolution<double> relaxed_compact_solver(const Instance& instance, int time_limit = 60, bool verbose = false);
