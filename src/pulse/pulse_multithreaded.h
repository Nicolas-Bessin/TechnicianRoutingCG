#pragma once

#include "pulse.h"

#include <mutex>

class PulseAlgorithmMultithreaded : virtual public PulseAlgorithm {
public:
    // Constructor
    PulseAlgorithmMultithreaded(Problem* problem, int delta, int pool_size) : PulseAlgorithm(problem, delta, pool_size) {}

    // Overload the pool update - we need to lock the pool
    void update_pool(double cost, const PartialPath& path);
    
    // Overloaded version of the pulse method - with mutex access to the solution pool
    void pulse(int vertex, int time, std::vector<int> quantities, double cost, const PartialPath& path);

    // Parallelized version of the pulse algorithm - calling the pulse on each forward neighbor of the vertex in parallel
    void pulse_parallel(int vertex, int time, std::vector<int> quantities, double cost, const PartialPath& path);

    // Parallelized version of the bounding phase - call the pulse_parallel on each vertex.
    void bound_parallel();

    // Parallelized version of the solving part
    int solve_parallel(double fixed_cost, double dual_value, bool use_maximisation_formulation);

protected:
    // Solution pool mutex
    std::mutex pool_mutex;

};