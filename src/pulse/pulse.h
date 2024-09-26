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

    void pulse(int v, int t, std::vector<int> q, double r, PartialPath& p);

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
};