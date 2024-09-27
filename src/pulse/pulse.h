#pragma once

#include "../../pathwyse/core/data/problem.h"
#include "pricing_problem/time_window_lunch.h"
#include "routes/route.h"

#include <array>
#include <vector>


struct PartialPath {
    std::vector<int> is_visited;
    std::vector<int> sequence;
};

// Return an empty path corresponding to a graph with N vertices
PartialPath EmptyPath(const int N);

PartialPath extend_path(const PartialPath& path, const int vertex);

class PulseAlgorithm {

public :
    // Constructor 
    PulseAlgorithm(Problem* problem);
    // Destructor
    ~PulseAlgorithm();

    // Reset the best path and best objective value (Used during the bounding phase)
    void reset();

    // Main pulse algorithm
    void pulse(int vertex, int time, std::vector<int> quantities, double cost, const PartialPath& path);

    // Bounding phase
    void bound(int delta);

    // Returns true if the bound is respected, that is, if we might reach a better solution
    bool check_bounds(int vertex, int time, double cost);

    // If going through the last vertex in p was a mistake, we rollback the choice
    bool rollback(int vertex, const PartialPath & path);

    // Full solving procedure, returns an error code (0 if everything went well)
    int solve(int delta);

    // Getters
    PartialPath get_best_path() {return best_path;}
    double get_best_objective() {return best_objective;}

private :
    // Underlying problem
    Problem* problem;
    // Origin and destination nodes
    int origin;
    int destination;
    // Number of nodes
    int N;
    // Number of capacities
    int K;
    // // Capacity resources
    // std::vector<Capacity*> capacities;
    // // Time resource
    // CustomTimeWindow *time;
    // // Objective
    // DefaultCost* objective;

    // Best path and best objective value
    PartialPath best_path;
    double best_objective;

    // Delta definition for the bounding matrix
    int delta = 5;

    // Matrix of bounds
    // bounds[i][j] is a lower bound on the best path from v_i to the destination using less than (j-1)*delta units of time
    std::vector<std::vector<double>> bounds;
};