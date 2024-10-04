#pragma once

#include <map>
#include <any>
#include <string>

#include "pricing_problem/pricing.h"

inline constexpr int ALL_RESOURCES_DOMINANCE = -1;


struct ColumnGenerationParameters {
    // General parameters
    int time_limit = 60; // (in seconds)
    double reduced_cost_threshold = 1e-6;
    bool verbose = false;
    int max_iterations = 1000;
    int max_consecutive_non_improvement = 5;
    bool compute_integer_solution = false;
    // Pathwyse related parameters
    int max_resources_dominance = ALL_RESOURCES_DOMINANCE;
    bool switch_to_cyclic_pricing = true;

    // Pulse related parameters
    int delta = 10;
    int solution_pool_size = 1000;

    // Stabilisation parameters
    double alpha = 0.5;
    bool use_stabilisation = false;

    // Pricing function
    std::string pricing_function = PRICING_PATHWYSE_BASIC;

    // ALl default values
    ColumnGenerationParameters() {};

    // Constructor : all values in the map are set, other set to default (see parameters.h)
    ColumnGenerationParameters(std::map<std::string, std::any>);

};


struct BranchAndPriceParameters : ColumnGenerationParameters {
    // Added parameters for the branch and price algorithm
    int max_depth = 100;
    int time_limit_per_node = 60;
    // All default values
    BranchAndPriceParameters() : ColumnGenerationParameters() {};
    // Constructor : all values in the map are set, other set to default (see parameters.h)
    BranchAndPriceParameters(std::map<std::string, std::any>);
};
