#pragma once

#include "../../pathwyse/core/data/problem.h"
#include "pricing_problem/time_window_lunch.h"

#include <array>
#include <vector>


struct PartialPath {
    std::vector<int> is_visited;
    std::vector<int> sequence;
};

// Return an empty path corresponding to a graph with N vertices
PartialPath EmptyPath(const int N) {
    return PartialPath {std::vector<int>(N, 0), std::vector<int>()};
}

PartialPath extend_path(const PartialPath& path, const int vertex) {
    PartialPath new_path = path;
    new_path.is_visited[vertex] = 1;
    new_path.sequence.push_back(vertex);
    return new_path;
}

class PulseSolver {

public :
    // Constructor 
    PulseSolver(Problem* problem);

    // Reset the best path and best objective value (Used during the bounding phase)
    void reset();

    // Main pulse algorithm
    void pulse(int v, int t, std::vector<int> q, double r, PartialPath& p);

    // Bounding phase
    void bound(int delta);

    // Returns true if the bound is respected, that is, if we might reach a better solution
    bool check_bounds(int v, int t, double r);

    bool rollback(int v, int t, double r, PartialPath p);

private :
    // Underlying problem
    Problem* problem;
    // Origin and destination nodes
    int origin;
    int destination;
    // Number of nodes
    int N;
    // Capacity resources
    std::vector<Capacity*> capacities;
    // Time resource
    CustomTimeWindow *time;
    // Objective
    DefaultCost* objective;

    // Best path and best objective value
    PartialPath best_path;
    double best_objective;

    // Delta definition for the bounding matrix
    int delta = 5;

    // Matrix of bounds
    // bounds[i][j] is a lower bound on the best path from v_i to the destination using less than (j-1)*delta units of time
    std::vector<std::vector<double>> bounds;
};