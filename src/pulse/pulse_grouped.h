#pragma once

#include "pulse.h"

// Extension of the PulseAlgorithm class to handle multiple vehicles with different subsets of feasible interventions on the same graph
class PulseAlgorithmWithSubsets : public PulseAlgorithm {
public:
    // Same methods as the PulseAlgorithm class

    // Constructor
    PulseAlgorithmWithSubsets(Problem* problem, int delta, int pool_size) : PulseAlgorithm(problem, delta, pool_size) {}

    // Sets the available interventions for the pulse feasibility check
    void set_available_interventions(std::vector<int> available_interventions);

    // Pulse methods is overloaded to handle the available interventions
    void pulse(int vertex, int time, std::vector<int> quantities, double cost, const PartialPath& path);

    // Reset is also overloaded to reset the available interventions
    void reset();

    // Only after the bounding, solve the problem for a vehicle with a given fixed cost, dual value, and available interventions
    int solve(double fixed_cost, double dual_value, std::vector<int> available_interventions);

protected:
    // Available interventions
    std::vector<int> available_interventions;
};