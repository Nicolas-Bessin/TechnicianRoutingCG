#include "branch_and_price.h"

#include "column_generation.h"

#include <vector>
#include <queue>
#include <chrono>

#define GLOBAL_TIME_LIMIT = 3600

BPNode RootNode(const std::vector<Route>& initial_routes){
    BPNode root;
    root.depth = 0;
    root.upper_bound = 0;
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
        using std::queue, std::vector;

        // First step : we create the root node from the initial routes
        std::vector<Route> routes = initial_routes;
        BPNode root_node = RootNode(initial_routes);
        queue<BPNode> node_queue;
        node_queue.push(root_node);
        // We also keep track of the best node & corresponding solution value
        BPNode best_node = BPNode(root_node);
        double best_obj = 0;

        // Keep track of the number of nodes explored and the depth
        int max_depth = 0;
        int nodes_explored = 0;

        // Main loop
        while(!node_queue.empty() && max_depth < 10) {
            // Get the top node in the queue and remove it 
            BPNode current_node = node_queue.front();
            node_queue.pop();

            // Solve this node
            CGResult result = column_generation(instance, current_node, routes, reduced_cost_threshold, time_limit_per_node, 100, true, false);
        }


        return;
}