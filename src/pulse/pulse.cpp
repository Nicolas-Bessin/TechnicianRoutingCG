#include "pulse.h"

#define FORWARD true

// Constructor
PulseSolver::PulseSolver(Problem* problem) {
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

void PulseSolver::pulse(int v, int t, std::vector<int>q, double r, PartialPath& p) {
    using std::vector;
    // Check the feasibility of the partial path
    bool feasible = true;
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
    if (rollback(v, t, r, p)) {
        return;
    }
    // Check if the destination node is reached

    // TODO

    
    // Extend the capacities
    for (int c = 0; c < capacities.size(); c++) {
        q[c] = capacities[c]->extend(q[c], p.sequence.back(), v, FORWARD);
    }
    // Extend the path
    PartialPath p_new = extend_path(p, v);

    // Pulse from all the forward neighbors
    auto neighbors = problem->getNeighbors(v, FORWARD);
    for (int i = 0; i < neighbors.size(); i++) {
        int r_new = objective->extend(r, v, neighbors[i], FORWARD);
        int t_new = time->extend(t, v, neighbors[i], FORWARD);
        pulse(neighbors[i], t_new, q, r_new, p_new);
    }

    return;
}