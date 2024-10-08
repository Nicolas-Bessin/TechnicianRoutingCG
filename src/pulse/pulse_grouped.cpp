#include "pulse_grouped.h"



/*
Extended version of the pulse algorithm that allows for multiple vehicles with the same o/d on the same graph
*/


void PulseAlgorithmWithSubsets::set_available_interventions(std::vector<int> available) {
    available_interventions = available;
}

void PulseAlgorithmWithSubsets::pulse(int vertex, int time, std::vector<int> quantities, double cost, const PartialPath& path) {
    using std::vector;
    using std::cout, std::endl;
    // Check the feasibility of the partial path
    bool feasible = true;
    // Check that the vertex is available
    feasible = feasible && available_interventions[vertex];
    // Is the path elementary ?
    feasible = feasible && !path.is_visited[vertex];
    for (int c = 0; c < K; c++) {
        feasible = feasible && problem->getRes(c)->isFeasible(quantities[c]);
    }
    feasible = feasible && problem->getRes(K)->isFeasible(time, vertex);
    if (!feasible) {
        //cout << "Pruning because of infeasibility at vertex " << vertex << " with time " << time << " and cost " << cost;
        //print_path_inline(path);
        return;
    }
    if (!check_bounds(vertex, time, cost) ) {
        //cout << "Pruning because of bound at vertex " << vertex << " with time " << time << ", bounding index " << get_bound_index(time, delta) << " and cost " << cost;
        //print_path_inline(path);
        return;
    }
    if (rollback(vertex, path)) {
        //cout << "Pruning because of rollback at vertex " << vertex << " with time " << time << " and cost " << cost;
        //print_path_inline(path);
        return;
    }

    
    // Extend the capacities
    for (int c = 0; c < K; c++) {
        int back = -1; // We don't care about the previous node for the capacities extension
        quantities[c] = problem->getRes(c)->extend(quantities[c], -1, vertex, FORWARD);
    }
    // Extend the path
    PartialPath p_new = extend_path(path, vertex);

    // Check if we are at the destination
    if (vertex == destination) {
        if (cost < best_objective) {
            best_objective = cost;
            best_path = p_new;
        }
        if (cost < pool_bound) {
            // Insert the new solution in the pool
            solutions.insert(std::pair<double, PartialPath>(cost, p_new));
            // If the pool is too large, we remove the worst solution
            if (solutions.size() > pool_size) {
                auto it = std::prev(solutions.end());
                solutions.erase(it);
            }
            // Update the pool bound
            pool_bound = solutions.rbegin()->first;
        return;
        }
    }

    // Pulse from all the forward neighborsj
    auto neighbors = problem->getNeighbors(vertex, FORWARD);
    for (int i = 0; i < neighbors.size(); i++) {
        double r_new = problem->getObj()->extend(cost, vertex, neighbors[i], FORWARD);
        int t_new = problem->getRes(K)->extend(time, vertex, neighbors[i], FORWARD);
        PulseAlgorithmWithSubsets::pulse(neighbors[i], t_new, quantities, r_new, p_new);
    }

    return;

}


void PulseAlgorithmWithSubsets::reset() {
    available_interventions = std::vector<int>(N, 1);
    PulseAlgorithm::reset();
}


int PulseAlgorithmWithSubsets::solve(double fixed_cost, double dual_value, std::vector<int> available) {
    reset();
    set_available_interventions(available);
    // Launch the pulse algorithm
    PartialPath path = EmptyPath(N);
    double initial_cost = fixed_cost - dual_value;
    pulse(origin, 0, std::vector<int>(K, 0), initial_cost, path);

    if(best_path.sequence.size() == 0) {
        return 1;
    }

    return 0;
}