#include "pulse.h"

#include "instance/constants.h"

#include "pricing_problem/time_window_lunch.h"

#include <chrono>

PartialPath EmptyPath(const int N) {
    PartialPath p{};
    p.is_visited = std::vector<bool>(N, false);
    p.sequence = std::vector<int>();
    p.start_times = std::vector<int>();
    return p;
}

PartialPath extend_path(const PartialPath& path, int vertex, int time) {
    PartialPath new_path = path;
    new_path.sequence.push_back(vertex);
    new_path.start_times.push_back(time);
    new_path.is_visited[vertex] = true;
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


inline int get_bound_index(int time, int delta) {
    return ceil( (double) (END_DAY - time) / (double) delta) - 1;
}


BoundData InfeasibleBound(const int N, const int K) {
    return BoundData {
        std::numeric_limits<double>::infinity(),
        EmptyPath(N),
        std::vector<int>(K, 0),
        END_DAY
    };
}

BoundData NonComputedBound(const int N, const int K) {
    return BoundData {
        -std::numeric_limits<double>::infinity(),
        EmptyPath(N),
        std::vector<int>(K, 0),
        END_DAY
    };
}

BoundData EmptyBound(const int N, const int K) {
    return BoundData {
        0,
        EmptyPath(N),
        std::vector<int>(K, 0),
        END_DAY
    };
}

int PulseAlgorithm::latest_start_time(int vertex, int initial_time, int next_vertex, int next_time) const {
    // Get the slack along edge (a, b)
    // Is there margin after the intervention - waiting time ?
    int travel_time = problem->getRes(K)->getArcCost(vertex, next_vertex);
    int duration = problem->getRes(K)->getNodeCost(vertex);
    int arrival_slack = std::max(0, next_time - (initial_time + duration + travel_time));
    // Is there margin to move the intervention later in the time window ?
    int end_time_a = problem->getRes(K)->getNodeUB(vertex); // In the problem, the time windows are [sw_i, ew_i - d_i] and we check the exact time point.
    int tw_slack = std::max(0, end_time_a - initial_time);
    // If intervention a is subject to the lunch constraint, we need to make sure we are not shifting it over lunch time
    int lunch_slack = std::numeric_limits<int>::max();
    bool has_lunch_constraint = dynamic_cast<CustomTimeWindow*>(problem->getRes(K))->hasLunchConstraint(vertex);
    if (has_lunch_constraint && initial_time + duration <= MID_DAY) {
            lunch_slack = MID_DAY - (initial_time + duration);
        }

    // The available slack is the minimum of the three
    int available_slack = std::min(arrival_slack, tw_slack);
    available_slack = std::min(available_slack, lunch_slack);

    int new_start_time = initial_time + available_slack;

    // Also check wether we could move the intervention to the afternoon
    if (has_lunch_constraint && initial_time + duration <= MID_DAY) {
        if(MID_DAY + duration + travel_time <= next_time && MID_DAY + duration <= end_time_a) {
            new_start_time = latest_start_time(vertex, MID_DAY, next_vertex, next_time);
        }
    }

    return new_start_time;
}


void PulseAlgorithm::update_bound(int vertex, int tau, double cost, PartialPath path, const std::vector<int>& quantities) {
    // Check if the path is feasible
    if (cost == std::numeric_limits<double>::infinity() || path.sequence.size() < 2){
        bounds[vertex][get_bound_index(tau, delta)] = InfeasibleBound(N, K);
        return;
    }
    // Compute the latest start time
    auto start_times = path.start_times;
    // Set the last time to END_DAY - we can introduce virtual waiting time at the destination
    start_times.back() = END_DAY;
    // We start backwards and add the available slack time to the start time of each node
    for (int i = path.sequence.size() - 2; i >= 0; i--) {
        // Update the latest start time
        int new_start_time = latest_start_time(path.sequence[i], start_times[i], path.sequence[i + 1], start_times[i + 1]);
        start_times[i] = new_start_time;
    }
    // The latest start time is the start time of the first node
    int latest_start_time = start_times[0];

    // Set the start times of this path to the latest start times
    path.start_times = start_times;

    // Update the bound
    bounds[vertex][get_bound_index(tau, delta)] = BoundData {
        cost,
        path,
        quantities,
        latest_start_time
    };

    return;
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
    bounds = std::vector<std::vector<BoundData>>(N, std::vector<BoundData>(num_bounds, NonComputedBound(N, K)));

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
            update_bound(v, tau, best_objective, best_path, best_quantities);
            reset();
        }
        bound_level++;
    }
    // Finally, the bound on the origin is -infinity, and the bound on the destination is 0
    for (int j = 0; j < num_bounds; j++) {
        bounds[origin][j] = NonComputedBound(N, K);
        bounds[destination][j] = EmptyBound(N, K);
    }
 
}


bool PulseAlgorithm::is_feasible(int vertex, int time, std::vector<int> quantities, double cost, const PartialPath & path) const {
    // Check the feasibility of the partial path
    bool feasible = true;
    // Is the path elementary ?
    feasible = feasible && !path.is_visited[vertex];
    for (int c = 0; c < K; c++) {
        feasible = feasible && problem->getRes(c)->isFeasible(quantities[c]);
    }
    feasible = feasible && problem->getRes(K)->isFeasible(time, vertex);
    return feasible;
}

bool PulseAlgorithm::check_bounds(int vertex, int time, double cost) const {
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
    return cost + bounds[vertex][j].cost < best_objective;
}


bool PulseAlgorithm::rollback(int vertex, const PartialPath & path) const {
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


bool PulseAlgorithm::splice(const PartialPath& path, int vertex, int time, double cost, std::vector<int> quantities) {
    BoundData& best_bound = bounds[vertex][get_bound_index(time, delta)];
    // If the bound has an empty path, we return false
    if (best_bound.path.sequence.size() == 0) {
        return false;
    }
    // Ensure that the first vertex of the bound is the current vertex
    if (best_bound.path.sequence[0] != vertex) {
        std::cerr << "Error: bound path does not start at the current vertex" << std::endl;
        return false;
    }
    PartialPath& extension = best_bound.path;
    // Check wether we can splice the current partia path with the best path in the pool
    // First step is checking wether the spliced path would be elementary
    bool elementary = true;
    for (int i = 0; i < N; i++) {
        elementary = elementary && (extension.is_visited[i] ? !path.is_visited[i] : true);
    }
    // If the path is not elementary, we return false
    if (!elementary) {
        return false;
    }
    // We then check the time feasibility
    if (time > best_bound.latest_start_time) {
        return false;
    }
    // We then check the resource feasibility
    for (int c = 0; c < K; c++) {
        if (quantities[c] + best_bound.quantities[c] > problem->getRes(c)->getUB()) {
            return false;
        }
    }

    // Here we can splice the path
    PartialPath new_path = path;
    new_path.sequence.insert(new_path.sequence.end(), extension.sequence.begin(), extension.sequence.end());
    // When the extension was set as a bound, its start times were set to the latest start times possible.
    // We can thus directly append the start times of the extension to the start times of the path
    // This might introduce some slack at the junction - but this slack will be removed when registering the bound
    new_path.start_times.insert(new_path.start_times.end(), extension.start_times.begin(), extension.start_times.end());
    // We also update the quantities
    std::vector<int> new_quantities = quantities;
    for (int c = 0; c < K; c++) {
        new_quantities[c] += best_bound.quantities[c];
    }
    // And the visited nodes
    for (int i = 0; i < N; i++) {
        new_path.is_visited[i] = path.is_visited[i] || extension.is_visited[i];
    }
    // We also update the cost
    cost += best_bound.cost;
    // new_path is now a full path to the destination, we can pass it to the update_pool method
    update_pool(cost, new_path, new_quantities);
    return true;
}
    

void PulseAlgorithm::update_pool(double cost, const PartialPath & path, const std::vector<int> & quantities) {
    if (cost < best_objective) {
            best_objective = cost;
            best_path = path;
            best_quantities = quantities;
        }
    // We only care about the quantities for the very best path
    if (cost < pool_bound) {
        // Insert the new solution in the pool
        solutions.insert(std::pair(cost, path));
        // If the pool is too large, we remove the worst solution
        if (solutions.size() > pool_size) {
            auto it = std::prev(solutions.end());
            solutions.erase(it);
        }
        // Update the pool bound
        pool_bound = solutions.rbegin()->first;
    }
        
    return;
}

void PulseAlgorithm::pulse(int vertex, int time, std::vector<int>quantities, double cost, const PartialPath& path) {
    using std::vector;
    using std::cout, std::endl;
    // Check the feasibility of the partial path
    if (!is_feasible(vertex, time, quantities, cost, path)) {
        return;
    }
    // Check the bounds
    if (!check_bounds(vertex, time, cost)) {
        return;
    }
    // Check if we need to rollback
    if (rollback(vertex, path)) {
        return;
    }   
    // Check if we can splice the path with the best path in the pool
    if (splice(path, vertex, time, cost, quantities)) {
        return;
    }   
    // Extend the capacities
    for (int c = 0; c < K; c++) {
        quantities[c] += problem->getRes(c)->getNodeCost(vertex);
    }
    // Extend the path
    PartialPath p_new = extend_path(path, vertex, time);

    // Check if we are at the destination
    if (vertex == destination) {
        update_pool(cost, p_new, quantities);
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


int PulseAlgorithm::solve(double fixed_cost, double dual_value) {
    // Launch the pulse algorithm
    reset();
    PartialPath path = EmptyPath(N);
    double initial_cost = fixed_cost - dual_value;
    pulse(origin, 0, std::vector<int>(K, 0), initial_cost, path);

    if(best_objective == std::numeric_limits<double>::infinity()) {
        return 1;
    }

    return 0;
}


int PulseAlgorithm::bound_and_solve(double fixed_cost, double dual_value) {
    // Launch the bounding phase
    reset();
    bound();
    return solve(fixed_cost, dual_value);
}




