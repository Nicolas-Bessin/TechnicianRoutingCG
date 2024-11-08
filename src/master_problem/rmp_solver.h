#pragma once

#include "instance/instance.h"
#include "master_problem/master.h"
#include "routes/route.h"
#include "master_problem/node.h"

/*
    Solve the relaxed restricted master problem associated with a given node
    @param instance : the instance of the problem
    @param routes : the vector of all routes (active or not)
    @param node : the node for which we want to solve the relaxed master problem (contains information on active routes)
    @return the solution to the relaxed master problem
*/
MasterSolution relaxed_RMP(const Instance& instance, const std::vector<Route>& routes, const BPNode& node);

/*
    Solve the integer restricted master problem associated with a given node
    @param instance : the instance of the problem
    @param routes : the vector of all routes (active or not)
    @param node : the node for which we want to solve the integer master problem (contains information on active routes)
    @return the solution to the integer master problem
*/
IntegerSolution integer_RMP(const Instance& instance, const std::vector<Route>& routes, const BPNode& node, int time_limit = -1, bool verbose = false);


/*
    Solve the integer restricted master problem associated with a given node using the Maximisation formulation
    @param instance : the instance of the problem
    @param routes : the vector of all routes (active or not)
    @param node : the node for which we want to solve the integer master problem (contains information on active routes)
    @return the solution to the integer master problem
*/
IntegerSolution integer_RMP_max(const Instance& instance, const std::vector<Route>& routes, const BPNode& node, int time_limit = -1, bool verbose = false);