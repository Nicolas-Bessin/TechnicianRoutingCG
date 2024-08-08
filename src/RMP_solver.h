#include "master.h"

#include "instance.h"
#include "route.h"
#include "branch_and_price.h"

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
IntegerSolution integer_RMP(const Instance& instance, const std::vector<Route>& routes, const BPNode& node);