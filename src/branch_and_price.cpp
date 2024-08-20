#include "branch_and_price.h"

#include "column_generation.h"

#include <vector>
#include <queue>
#include <chrono>
#include <iostream>

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

double compute_x(int i, int j, int v,const MasterSolution& master_solution, const std::vector<Route>& routes, const Instance& instance){    
    double x_ijv = 0;
    for (int r = 0; r < routes.size(); r++){
        if (master_solution.coefficients[r] == 0) continue;
        if (routes[r].vehicle_id != v) continue;
        x_ijv += master_solution.coefficients[r] * routes[r].route_edges[i][j];
    }
    return x_ijv;
}

std::tuple<int, int, int> find_first_valid_cut(const MasterSolution& master_solution, const std::vector<Route>& routes, const Instance& instance) {
    // Get the problem sizes
    int n_nodes = instance.nodes.size();
    int n_vehicles = instance.vehicles.size();
    for (int i = 0; i < n_nodes; i++) {
        for (int j = 0; j < n_nodes; j++) {
            for (int v = 0; v < n_vehicles; v++) {
                double x_ijv = compute_x(i, j, v, master_solution, routes, instance);
                if (0.05 < x_ijv && x_ijv < 0.95) {
                    std::cout << "Found a valid cut at (" << i << ", " << j << ", " << v << ") with value " << x_ijv << std::endl;
                    return std::make_tuple(i, j, v);
                }
            }
        }
    }
    return std::make_tuple(-1, -1, -1);
}

void branch_and_price(
    const Instance& instance, 
    const std::vector<Route>& initial_routes,
    double reduced_cost_threshold,
    int time_limit_per_node,
    int max_cg_iter,
    int max_depth,
    bool cyclic_pricing,
    bool verbose
    ){
        using std::queue, std::vector;
        using std::cout, std::endl;

        // Get the problem sizes
        int n_nodes = instance.nodes.size();
        int n_vehicles = instance.vehicles.size();
        // First step : we create the root node from the initial routes
        std::vector<Route> routes = initial_routes;
        BPNode root_node = RootNode(initial_routes);
        queue<BPNode> node_queue;
        node_queue.push(root_node);
        // We also keep track of the best node & corresponding solution value
        BPNode best_node = BPNode(root_node);
        double best_obj = 0;

        // Keep track of the number of nodes explored and the depth
        int depth = 0;
        int nodes_explored = 0;

        // Main loop
        while(!node_queue.empty() && depth <= max_depth) {
            // Get the top node in the queue and remove it 
            BPNode current_node = node_queue.front();
            node_queue.pop();

            cout << "---------------------------------" << endl;
            cout << "Exploring node at depth " << current_node.depth << " with " << current_node.active_routes.size() << " active routes" << endl;
            cout << "Number of upper bound cuts : " << current_node.upper_bound_cuts.size();
            cout << " - Number of lower bound cuts : " << current_node.lower_bound_cuts.size() << endl;

            // Update the lower bound to be the best solution found so far
            // If we find that this node can't beat the best solution, we can skip it
            current_node.lower_bound = best_obj;

            nodes_explored += 1;
            depth = std::max(depth, current_node.depth);

            // For the root node, do not set a time limit (we set it to 1000 times the normal time limit)
            int time_limit;
            if (current_node.depth == 0) {
                time_limit = 600;
            } else {
                time_limit = time_limit_per_node;
            }

            // Solve this node
            CGResult result = column_generation(instance, current_node, routes, reduced_cost_threshold, time_limit, max_cg_iter, cyclic_pricing, true, false);

            // If the returned relaxed solution is tagged as non feasible, it means the cuts introduced to this node are non feasible
            if (!result.master_solution.is_feasible) {
                cout << "Cuts are non feasible - pruning this node" << endl;
                continue;
            }

            // If we returned an Integer Solution tagged as non feasible, it means we got a node upper bound worse than the global lower bound
            if (!result.integer_solution.is_feasible) {
                cout << "Upper bound is worse than best know solution - pruning this node" << endl;
                continue;
            }

            double integer_objective = result.integer_solution.objective_value;
            double relaxed_objective = result.master_solution.objective_value;

            // If we improved the best solution, keep the node in memory
            if (integer_objective > best_obj) {
                cout << "New best solution found : " << integer_objective << endl;
                best_obj = integer_objective;
                best_node = current_node;
            }
            // If the integer solution has the same value as the relaxed solution : no need to branch
            if (abs(integer_objective - relaxed_objective) < 1e-3) {
                cout << "No need to branch - integer solution is optimal" << endl;
                continue;
            }

            // We need to branch - find the first non integer x_ijv (for now)
            auto ijv = find_first_valid_cut(result.master_solution, routes, instance);

            // Create the left (UB) and right (LB) node
            BPNode left_node = BPNode(current_node);
            left_node.upper_bound_cuts.insert(ijv);
            left_node.upper_bound = relaxed_objective;
            left_node.depth += 1;
            node_queue.push(left_node);

            BPNode right_node = BPNode(current_node);
            right_node.lower_bound_cuts.insert(ijv);
            right_node.upper_bound = relaxed_objective;
            right_node.depth += 1;
            node_queue.push(right_node);

        }


        return;
}


std::set<std::tuple<int, int, int>> routes_to_required_edges(const std::vector<Route>& routes){
    std::set<std::tuple<int, int, int>> required_edges;
    for (const Route& route : routes){
        for (int i = 0; i < route.route_edges.size(); i++){
            for (int j = 0; j < route.route_edges.size(); j++){
                if (route.route_edges[i][j] == 1){
                    required_edges.insert(std::make_tuple(i, j, route.vehicle_id));
                }
            }
        }
    }
    return required_edges;
}