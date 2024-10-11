#pragma once

#include "../../pathwyse/core/data/problem.h"
#include "pricing_problem/time_window_lunch.h"
#include "routes/route.h"

#include <vector>
#include <map>



struct PartialPath {
    std::vector<bool> is_visited;
    std::vector<int> sequence;
    std::vector<int> start_times;
};

// Return an empty path corresponding to a graph with N vertices
PartialPath EmptyPath(const int N);

// Extend a path by adding a vertex to it
PartialPath extend_path(const PartialPath& path, int vertex, int time);

struct BoundData {
    double cost;
    PartialPath path;
    std::vector<int> quantities;
    int latest_start_time;
};

// Return a bound with a cost of +infinity
BoundData InfeasibleBound(const int N, const int K);

// Return a bound with a cost of -infinity
BoundData NonComputedBound(const int N, const int K);

// Return a bound with a cost of 0
BoundData EmptyBound(const int N, const int K);

class PulseAlgorithm {

public :
    // Constructor
    // @param problem : the underlying problem - will not be deleted
    // @param delta : the time step for the bounding phase
    // @param pool_size : the size of the solution pool
    PulseAlgorithm(Problem* problem, int delta, int pool_size);

    // Reset the best path and best objective value (Used during the bounding phase)
    void reset();

    // Feasibility check for the pulse algorithm
    bool is_feasible(int vertex, int time, std::vector<int> quantities, double cost, const PartialPath & path) const;

    // Returns true if the bound is respected, that is, if we might reach a better solution
    bool check_bounds(int vertex, int time, double cost) const;

    // If going through the last vertex in p was a mistake, we rollback the choice
    bool rollback(int vertex, const PartialPath & path) const;

    // Try to splice the path with the best path in the pool, returns true if the path was spliced
    bool splice(const PartialPath & path, int vertex, int time, double cost, std::vector<int> quantities);

    // Main pulse algorithm
    void pulse(int vertex, int time, std::vector<int> quantities, double cost, const PartialPath & path);

    // Update the conditionnal lower bound on the best path from vertex to the destination
    void update_bound(int vertex, int tau, double cost, PartialPath path, std::vector<int> quantities);

    // Update the solution pool with a new solution
    void update_pool(double cost, const PartialPath & path);

    // Bounding phase
    void bound();

    // Only the solving part of the pulse algorithm
    int solve(double fixed_cost, double dual_value);
    
    // Full solving procedure, returns an error code (0 if everything went well)
    int bound_and_solve(double fixed_cost, double dual_value);

    // Getters
    PartialPath get_best_path() {return best_path;}
    double get_best_objective() {return best_objective;}
    auto get_solution_pool() {return solutions;}

protected :
    // Underlying problem
    Problem* problem;
    // Origin and destination nodes
    const int origin;
    const int destination;
    // Number of nodes
    const int N;
    // Number of capacities
    const int K;

    // Best path and best objective value
    PartialPath best_path;
    double best_objective;

    // Size of the solution pool
    const int pool_size = 10;
    // Value of the worst solution in the pool - we only insert a new solution if it is better than this    
    double pool_bound = std::numeric_limits<double>::infinity();
    // Pool of solutions
    std::multimap<double, PartialPath> solutions;

    // Delta definition for the bounding matrix
    const int delta = 5;

    // Matrix of bounds
    // bounds[i][j] is a lower bound on the best path from v_i to the destination using less than (j-1)*delta units of time
    std::vector<std::vector<BoundData>> bounds;
};




