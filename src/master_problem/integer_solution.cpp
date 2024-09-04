#include "integer_solution.h"

#include "master_problem/rmp_solver.h"

#include <iostream>


IntegerSolution solve_integer_master_problem(
    const Instance& instance, 
    const std::vector<Route>& routes, 
    const BPNode& node, 
    bool verbose){

    using std::vector, std::set;

    // First step: create a new node where we removed the duplicates routes from the active pool
    BPNode node_no_duplicates = node;
    // Create a mask of evert route that has a duplictae earlier in the active routes
    vector<bool> has_duplicate(routes.size(), false);
    for (int i = 0; i < routes.size(); i++){
        for (int j = i+1; j < routes.size(); j++){
            if (routes[i] == routes[j]){
                has_duplicate[j] = true;
            }
        }
    }
    // Remove the duplicates from the active routes
    set<int> new_active_routes;
    for (int i : node.active_routes){
        if (!has_duplicate[i]){
            new_active_routes.insert(i);
        }
    }
    node_no_duplicates.active_routes = new_active_routes;

    std::cout << "Computing integer solution for node with " << node_no_duplicates.active_routes.size() << " active routes" << std::endl;
    // Solve the relaxed RMP
    return integer_RMP(instance, routes, node_no_duplicates, verbose);
}