#include "pulse.h"

#include "instance/constants.h"


#define FORWARD true

// Constructor
PulseAlgorithm::PulseAlgorithm(Problem* problem) {
    this->problem = problem;
    this->origin = problem->getOrigin();
    this->destination = problem->getDestination();
    this->N = problem->getNumNodes();
    // problem->getResources contains all the resources, including the Time resource (at the end)
    for (int i = 0; i < problem->getResources().size() - 1; i++) {
        this->capacities.push_back(dynamic_cast<Capacity*>(problem->getResources()[i]));
    }
    // Time resource
    this->time = dynamic_cast<CustomTimeWindow*>(problem->getResources().back());
    // Objective
    this->objective = dynamic_cast<DefaultCost*>(problem->getObj());
}

void PulseAlgorithm::reset() {
    best_objective = std::numeric_limits<double>::infinity();
    best_path = EmptyPath(N);
}


void PulseAlgorithm::bound(int delta) {
    this->delta = delta;

    // Initialize the bounds matrix
    // We have one bounding level in terms of time for each END_DAY - (j-1) * delta
    // Thus, bound[v][j] is the best objective value that can be achieved starting from vertex v with available time (j+) * delta
    // Or equivalently, the best objective value that can be achieved starting from vertex v at time END_DAY - (j + 1) * delta
    int num_bounds = ceil( (double) END_DAY / delta) - 1;

    if (num_bounds <= 0) {
        std::cerr << "Error: delta is too large" << std::endl;
        return;
    }
    bounds = std::vector<std::vector<double>>(N, std::vector<double>(num_bounds, std::numeric_limits<double>::infinity()));

    // Begin the bounding process with 0 time available (i.e. launch pulse from time END_DAY)
    int tau = END_DAY;

    int bound_level = 0;

    while (tau > 0) {
        // Add delta to the available time
        tau -= delta;
        // Launch a pulse algorithm from every vertex that is not the destination or the origin
        for (int v = 0; v < N; v++) {
            if (v == origin || v == destination) continue;

            // Start the pulse algorithm from an empty path
            PartialPath p = EmptyPath(N);
            pulse(v, tau, std::vector<int>(capacities.size(), 0), 0, p);
            // If the best path is empty, it means that no path was found, thus the bound has to be +infinity
            if (best_path.sequence.size() == 0) {
                bounds[v][bound_level] = std::numeric_limits<double>::infinity();
            } else {
                bounds[v][bound_level] = best_objective;
            }
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


bool PulseAlgorithm::check_bounds(int v, int t, double r) {
    // We want to find the lowest j such that END_DAY - (j-1) * delta <= t
    // That is, j = ceil((END_DAY - t) / delta) - 1
    int j = ceil((END_DAY - t) / delta) - 1;
    if (j < 0) {
        return false;
    }
    // We there is no bound available for this vertex, we return true (this means that the time consumption is too low for now)
    if (j >= bounds[v].size()) {
        return true;
    }
    // We return true if the value along the current path + the lowest we can achieve to the destination is less than the best objective
    // That is, we return true if we can potentially improve the best objective starting from the current path
    return r + bounds[v][j] < best_objective;
}

bool PulseAlgorithm::rollback(int v, PartialPath p) {
    // First step is checking the lenght of the path - rollback is only possible for pathes of length at least 2
    if (p.sequence.size() < 2) {
        return false;
    }
    // By construction, the ony thing we need to check is wether removing the last vertex of the path gives a better cost
    // - Thanks to the triangular inequality, the time along the path when removing the last vertex is lower
    // - By construction, the resource consumptions are also lower
    // - By construction, the rollback path is strictly included in the longer path
    int v_last = p.sequence[p.sequence.size() - 1];
    int v_prev = p.sequence[p.sequence.size() - 2];
    // Compute the cost of going from v_prev to v directly
    int r_new = objective->getArcCost(v_prev, v);
    // Compute the cost of going from v_prev to v_last and then to v
    int r_old = objective->getArcCost(v_prev, v_last) + objective->getNodeCost(v_last) + objective->getArcCost(v_last, v);
    // If the cost is better, we want to rollback the choice of going throug v_last
    return r_new <= r_old;
}

void PulseAlgorithm::pulse(int v, int t, std::vector<int>q, double r, PartialPath& p) {
    using std::vector;
    // Check the feasibility of the partial path
    bool feasible = true;
    // Is the path elementary ?
    feasible = feasible && p.is_visited[v] == 0;
    for (int c = 0; c < capacities.size(); c++) {
        feasible = feasible && capacities[c]->isFeasible(q[c]);
    }
    feasible = feasible && time->isFeasible(t, v);
    if (!feasible) {
        return;
    }
    if (!check_bounds(v, t, r) ) {
        return;
    }
    if (rollback(v, p)) {
        return;
    }

    
    // Extend the capacities
    for (int c = 0; c < capacities.size(); c++) {
        q[c] = capacities[c]->extend(q[c], p.sequence.back(), v, FORWARD);
    }
    // Extend the path
    PartialPath p_new = extend_path(p, v);

    // Check if we are at the destination
    if (v == destination) {
        if (r < best_objective) {
            best_objective = r;
            best_path = p_new;
        }
        return;
    }

    // Pulse from all the forward neighborsj
    auto neighbors = problem->getNeighbors(v, FORWARD);
    for (int i = 0; i < neighbors.size(); i++) {
        int r_new = objective->extend(r, v, neighbors[i], FORWARD);
        int t_new = time->extend(t, v, neighbors[i], FORWARD);
        pulse(neighbors[i], t_new, q, r_new, p_new);
    }

    return;
}

PartialPath PulseAlgorithm::solve(int delta) {

    // Launch the bounding phase
    reset();
    bound(delta);

    // Launch the pulse from the origin
    reset();
    PartialPath p = EmptyPath(N);
    pulse(origin, 0, std::vector<int>(capacities.size(), 0), 0, p);

    // Return the best path found
    return best_path;
}