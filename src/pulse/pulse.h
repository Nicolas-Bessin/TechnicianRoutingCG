#pragma once

#include "../../pathwyse/core/data/problem.h"
#include "pricing_problem/time_window_lunch.h"
#include "routes/route.h"

#include <array>
#include <vector>
#include <map>

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
    // @param problem : the underlying problem - will not be deleted
    // @param delta : the time step for the bounding phase
    // @param pool_size : the size of the solution pool
    PulseAlgorithm(Problem* problem, int delta, int pool_size, double fixed_cost);

    // Reset the best path and best objective value (Used during the bounding phase)
    void reset();

    // Main pulse algorithm
    void pulse(int vertex, int time, std::vector<int> quantities, double cost, const PartialPath& path);

    // Bounding phase
    void bound();

    // Returns true if the bound is respected, that is, if we might reach a better solution
    bool check_bounds(int vertex, int time, double cost);

    // If going through the last vertex in p was a mistake, we rollback the choice
    bool rollback(int vertex, const PartialPath & path);

    // Full solving procedure, returns an error code (0 if everything went well)
    int solve(int delta);

    // Getters
    PartialPath get_best_path() {return best_path;}
    double get_best_objective() {return best_objective;}
    auto get_solution_pool() {return solutions;}

private :
    // Underlying problem
    Problem* problem;
    // Origin and destination nodes
    const int origin;
    const int destination;
    // Number of nodes
    const int N;
    // Number of capacities
    const int K;

    // Fixed cost of the vehicle
    const double fixed_cost;

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
    std::vector<std::vector<double>> bounds;
};