#include "pulse.h"

#include "instance/constants.h"

#include <chrono>


PartialPath EmptyPath(const int N) {
    PartialPath p;
    p.is_visited = std::vector<int>(N, 0);
    p.sequence = std::vector<int>();
    return p;
}

PartialPath extend_path(const PartialPath& path, const int vertex) {
    PartialPath new_path = path;
    new_path.sequence.push_back(vertex);
    new_path.is_visited[vertex] = 1;
    return new_path;
}

void print_path_inline(const PartialPath& path) {
    std::cout << " - Path: [";
    for (int i = 0; i < path.sequence.size(); i++) {
        std::cout << path.sequence[i] << ", ";
    }
    std::cout << "]" <<  std::endl;
}

// Constructor
PulseAlgorithm::PulseAlgorithm(Problem* problem, int delta, int pool_size) :
    problem(problem),
    origin(problem->getOrigin()),
    destination(problem->getDestination()),
    N(problem->getNumNodes()),
    K(problem->getNumRes() - 1),
    delta(delta),
    pool_size(pool_size)
{
    best_objective = std::numeric_limits<double>::infinity();
    pool_bound = std::numeric_limits<double>::infinity();
    best_path = EmptyPath(N);

    // If the pool if of size 0, we tell the user there is a problem
    if (pool_size <= 0) {
        throw std::invalid_argument("Error: pool size must be strictly positive");
    }
}


void PulseAlgorithm::reset() {
    best_objective = std::numeric_limits<double>::infinity();
    best_path = EmptyPath(N);
    // Reset the pool
    pool_bound = std::numeric_limits<double>::infinity();
    solutions.clear();
}


int get_bound_index(int time, int delta) {
    return ceil( (double) (END_DAY - time) / (double) delta) - 1;
}

void PulseAlgorithm::bound() {
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
            pulse(v, tau, std::vector<int>(K, 0), 0, p);
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

bool PulseAlgorithm::check_bounds(int vertex, int time, double cost) {
    using std::cout, std::endl;
    // We want to find the lowest j such that END_DAY - (j-1) * delta <= t
    // That is, j = ceil((END_DAY - t) / delta) - 1
    int j = get_bound_index(time, delta);
    if (j < 0) {
        return false;
    }
    // We there is no bound available for this vertex, we return true (this means that the time consumption is too low for now)
    if (j >= bounds[vertex].size()) {
        return true;
    }
    // We return true if the value along the current path + the lowest we can achieve to the destination is less than the best objective
    // That is, we return true if we can potentially improve the best objective starting from the current path
    return cost + bounds[vertex][j] < best_objective;
}

bool PulseAlgorithm::rollback(int vertex, const PartialPath & path) {
    // First step is checking the lenght of the path - rollback is only possible for pathes of length at least 2
    if (path.sequence.size() < 2) {
        return false;
    }
    if (path.sequence.size() == 2 && path.sequence[0] == origin && vertex == destination) {
        return false;
    }
    // By construction, the ony thing we need to check is wether removing the last vertex of the path gives a better cost
    // - Thanks to the triangular inequality, the time along the path when removing the last vertex is lower
    // - By construction, the resource consumptions are also lower
    // - By construction, the rollback path is strictly included in the longer path
    int v_last = path.sequence[path.sequence.size() - 1];
    int v_prev = path.sequence[path.sequence.size() - 2];
    // Compute the cost of going from v_prev to v directly
    int r_new = problem->getObj()->getArcCost(v_prev, vertex);
    // Compute the cost of going from v_prev to v_last and then to v
    int r_old = problem->getObj()->getArcCost(v_prev, v_last) + problem->getObj()->getNodeCost(v_last) + problem->getObj()->getArcCost(v_last, vertex);
    // If the cost is better, we want to rollback the choice of going throug v_last
    return r_new <= r_old;
}

void PulseAlgorithm::pulse(int vertex, int time, std::vector<int>quantities, double cost, const PartialPath& path) {
    using std::vector;
    using std::cout, std::endl;
    // Check the feasibility of the partial path
    bool feasible = true;
    // Is the path elementary ?
    feasible = feasible && path.is_visited[vertex] == 0;
    for (int c = 0; c < K; c++) {
        feasible = feasible && problem->getResources()[c]->isFeasible(quantities[c]);
    }
    feasible = feasible && problem->getResources()[K]->isFeasible(time, vertex);
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
        quantities[c] = problem->getResources()[c]->extend(quantities[c], -1, vertex, FORWARD);
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
        int t_new = problem->getResources()[K]->extend(time, vertex, neighbors[i], FORWARD);
        pulse(neighbors[i], t_new, quantities, r_new, p_new);
    }

    return;
}

int PulseAlgorithm::bound_and_solve() {
    // Launch the bounding phase
    reset();
    bound();
    // Launch the pulse from the origin
    reset();
    PartialPath path = EmptyPath(N);
    double initial_cost = problem->getObj()->getNodeCost(origin);
    pulse(origin, 0, std::vector<int>(K, 0), initial_cost, path);

    if(best_path.sequence.size() == 0) {
        return 1;
    }

    return 0;
}


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
    feasible = feasible && path.is_visited[vertex] == 0;
    for (int c = 0; c < K; c++) {
        feasible = feasible && problem->getResources()[c]->isFeasible(quantities[c]);
    }
    feasible = feasible && problem->getResources()[K]->isFeasible(time, vertex);
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
        quantities[c] = problem->getResources()[c]->extend(quantities[c], -1, vertex, FORWARD);
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
        int t_new = problem->getResources()[K]->extend(time, vertex, neighbors[i], FORWARD);
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
    double initial_cost = fixed_cost + dual_value;
    pulse(origin, 0, std::vector<int>(K, 0), initial_cost, path);

    if(best_path.sequence.size() == 0) {
        return 1;
    }

    return 0;
}


Route convert_path_to_route(double rc, const PartialPath& path, const Instance& instance, const Vehicle& vehicle) {
    using std::vector;
    // Get all the info we need to build a Route object
    double total_cost = vehicle.cost;
    int total_duration = 0;
    vector<int> id_sequence;
    vector<int> is_in_route(instance.nodes.size(), 0);
    vector<vector<int>> route_edges(instance.nodes.size(), vector<int>(instance.nodes.size(), 0));

    // Sequence in terms of the vehicle's interventions (not the true node ids)
    auto tour = path.sequence;    

    for (int i = 0; i < tour.size() - 1; i++) {
        int true_i = i == 0 ? vehicle.depot : vehicle.interventions[tour[i]];
        int true_j = i+1 == tour.size()-1 ? vehicle.depot : vehicle.interventions[tour[i + 1]];
        // Update the sequence of interventions
        id_sequence.push_back(true_i);
        // Update the edge matrix
        route_edges[true_i][true_j] = 1;
        // Update the is_in_route
        is_in_route[true_i] = 1;
        // Get the duration, and distance between the two interventions
        int duration = instance.nodes[true_i].duration;
        int distance = instance.distance_matrix[true_i][true_j];
        // Update the running total cost & duration
        total_cost += instance.cost_per_km * distance;
        total_duration += duration;        
    }
    // Add the checks related to the last intervention
    int true_last = vehicle.depot;
    id_sequence.push_back(true_last);
    is_in_route[true_last] = 1;

    // Create the Route object
    return Route{
        vehicle.id,
        total_cost,
        rc,
        total_duration,
        id_sequence,
        is_in_route,
        route_edges
    };

}