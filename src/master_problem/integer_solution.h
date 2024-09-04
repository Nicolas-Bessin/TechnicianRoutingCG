#pragma once

#include "instance/instance.h"
#include "master_problem/node.h"
#include "master_problem/master.h"
#include "routes/route.h"

#include <vector>


/*
    Basic interface for solving the integer master problem associated with a given node  

    This implementation removes duplicate routes from the active routes and then calls the integer_RMP function
      
    @param instance : the instance of the problem
    @param routes : the vector of all routes (active or not)
    @param node : the node for which we want to solve the integer master problem (contains information on active routes)
    @return the solution to the integer master problem
*/
IntegerSolution solve_integer_master_problem(const Instance& instance, const std::vector<Route>& routes, const BPNode& node, bool verbose = false);

