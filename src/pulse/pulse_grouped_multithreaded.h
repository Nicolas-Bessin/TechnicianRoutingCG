#pragma once

#include "pulse_grouped.h"
#include "pulse_multithreaded.h"


class PulseAlgorithmMultithreadedGrouped : public PulseAlgorithmMultithreaded, public PulseAlgorithmWithSubsets {
public:
    // Constructor
    PulseAlgorithmMultithreadedGrouped(Problem* problem, int delta, int pool_size);

    // Overloaded version of the pulse method - with mutex access to the solution pool
    void pulse(int vertex, int time, std::vector<int> quantities, double cost, const PartialPath& path);

    // Parallelized version of the pulse algorithm - calling the pulse on each forward neighbor of the vertex in parallel
    void pulse_parallel(int vertex, int time, std::vector<int> quantities, double cost, const PartialPath& path);

    // Parallelized version of the bounding phase - call the pulse_parallel on each vertex.
    void bound();

    // Parallelized version of the solving part
    int solve(double fixed_cost, double dual_value, std::vector<int> available_interventions);

};