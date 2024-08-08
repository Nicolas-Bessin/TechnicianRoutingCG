#include "branch_and_price.h"

#include "column_generation.h"


BPNode RootNode(const std::vector<Route>& initial_routes){
    BPNode root;
    root.depth = 0;
    root.upper_bound = 0;
    root.lower_bound = 0;
    for (int i = 0; i < initial_routes.size(); i++){
        root.active_routes.insert(i);
    }
    return root;
}

void branch_and_price(
    const Instance& instance, 
    const std::vector<Route>& initial_routes,
    double reduced_cost_threshold,
    int time_limit_per_node,
    int max_depth,
    bool compute_integer_solution,
    bool verbose
    ){

        return;
}