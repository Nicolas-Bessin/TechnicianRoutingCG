#include "pulse_multithreaded.h"


#include <thread>


void PulseAlgorithmMultithreaded::bound_parallel() {
    // Initialize the bounds matrix
    // We have one bounding level in terms of time for each END_DAY - (j-1) * delta
    // Thus, bound[v][j] is the best objective value that can be achieved starting from vertex v with available time (j+) * delta
    // Or equivalently, the best objective value that can be achieved starting from vertex v at time END_DAY - (j + 1) * delta
    int num_bounds = ceil((double) END_DAY / (double) delta);

    if (num_bounds <= 0) {
        std::cerr << "Error: delta is too large" << std::endl;
        return;
    }
    bounds = std::vector<std::vector<double>>(N, std::vector<double>(num_bounds, - std::numeric_limits<double>::infinity()));

    // Begin the bounding process with 0 time available (i.e. launch pulse from time END_DAY)
    int tau = END_DAY;

    int bound_level = 0;

    while (tau > 0) {
        // Add delta to the available time
        tau -= delta;
        // std::cout << "-----------------------------------" << std::endl;
        // std::cout << "Bounding level " << bound_level << " with initial time " << tau << std::endl;
        // Launch a pulse algorithm from every vertex that is not the destination or the origin
        for (int v = 0; v < N; v++) {
            if (v == origin || v == destination) continue;

            // Start the pulse algorithm from an empty path
            PartialPath p = EmptyPath(N);
            pulse_parallel(v, tau, std::vector<int>(K, 0), 0, p);
            // If no path was found, the objective will have value +inf and thus bound is +infinity
            bounds[v][bound_level] = best_objective;
            reset();
        }
        bound_level++;
    }
    // Finally, the bound on the origin is -infinity, and the bound on the destination is 0
    for (int j = 0; j < num_bounds; j++) {
        bounds[origin][j] = -std::numeric_limits<double>::infinity();
        bounds[destination][j] = 0;
    }
}


void PulseAlgorithmMultithreaded::update_pool(double cost, const PartialPath& path) {
    // Acquire the lock
    pool_mutex.lock();
    if (cost < best_objective) {
        best_objective = cost;
        best_path = path;
    }
    if (cost < pool_bound) {
        // Insert the new solution in the pool
        solutions.insert(std::pair<double, PartialPath>(cost, path));
        // If the pool is too large, we remove the worst solution
        if (solutions.size() > pool_size) {
            auto it = std::prev(solutions.end());
            solutions.erase(it);
        }
        // Update the pool bound
        pool_bound = solutions.rbegin()->first;;
    }
    // Release the lock
    pool_mutex.unlock();
}


void PulseAlgorithmMultithreaded::pulse(int vertex, int time, std::vector<int>quantities, double cost, const PartialPath& path) {
    using std::vector;
    using std::cout, std::endl;
    // Check the feasibility of the partial path
    if (!is_feasible(vertex, time, quantities, cost, path)) {
        return;
    }
    if (!check_bounds(vertex, time, cost) ) {
        return;
    }
    if (rollback(vertex, path)) {
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
        update_pool(cost, p_new);
    }

    // Pulse from all the forward neighborsj
    auto neighbors = problem->getNeighbors(vertex, FORWARD);
    for (int i = 0; i < neighbors.size(); i++) {
        double r_new = problem->getObj()->extend(cost, vertex, neighbors[i], FORWARD);
        int t_new = problem->getRes(K)->extend(time, vertex, neighbors[i], FORWARD);
        pulse(neighbors[i], t_new, quantities, r_new, p_new);
    }

    return;
}


void PulseAlgorithmMultithreaded::pulse_parallel(int vertex, int time, std::vector<int> quantities, double cost, const PartialPath& path) {
    using std::vector;
    using std::cout, std::endl;
    // Check the feasibility of the partial path
    if (!is_feasible(vertex, time, quantities, cost, path)) {
        return;
    }
    if (!check_bounds(vertex, time, cost) ) {
        return;
    }
    if (rollback(vertex, path)) {
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
        std::cerr << "Error: destination reached in pulse_parallel" << std::endl;
        return;
    }

    // Parallelize the pulse algorithm to all the forward neighbors
    auto neighbors = problem->getNeighbors(vertex, FORWARD);
    vector<std::thread> threads(neighbors.size());
    for (int i = 0; i < neighbors.size(); i++) {
        int t_new = problem->getRes(K)->extend(time, vertex, neighbors[i], FORWARD);
        double r_new = problem->getObj()->extend(cost, vertex, neighbors[i], FORWARD);
        threads[i] = std::thread(&PulseAlgorithmMultithreaded::pulse, this, neighbors[i], t_new, quantities, r_new, p_new);
    }
    // Wait for completion
    for (int i = 0; i < neighbors.size(); i++) {
        threads[i].join();
    }

    return;
}


int PulseAlgorithmMultithreaded::solve_parallel(double fixed_cost, double dual_value) {
    // Launch the pulse algorithm
    reset();
    PartialPath path = EmptyPath(N);
    double initial_cost = fixed_cost - dual_value;
    PulseAlgorithmMultithreaded::pulse_parallel(origin, 0, std::vector<int>(K, 0), initial_cost, path);

    if(best_path.sequence.size() == 0) {
        return 1;
    }

    return 0;
}