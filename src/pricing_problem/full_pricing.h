#pragma once


#include "instance/instance.h"
#include "routes/route.h"
#include "master_problem/master.h"

#include <vector>
#include <random>

inline constexpr int RANDOM_SEED = -1;


inline constexpr std::string PRICING_PATHWYSE_BASIC = "basic_pathwyse";
std::vector<Route> full_pricing_problems_basic(
    const DualSolution & solution,
    const Instance & instance,
    const std::vector<int> & vehicle_order,
    bool using_cyclic_pricing,
    int n_ressources_dominance
);



inline constexpr std::string PRICING_DIVERSIFICATION = "diversification";
/*
    Generates a new set of route based on the diversitication algorithm presented in Dupin et al. (2021) 
*/
std::vector<Route> full_pricing_problems_diversification(
    const DualSolution & solution,
    const Instance & instance,
    const std::vector<int> & vehicle_order,
    bool using_cyclic_pricing,
    int n_ressources_dominance,
    int seed = RANDOM_SEED
);

inline constexpr std::string PRICING_CLUSTERING = "clustering";
std::vector<Route> full_pricing_problems_clustering(
    const DualSolution & solution,
    const Instance & instance,
    const std::vector<int> &vehicle_order,
    bool using_cyclic_pricing,
    int n_ressources_dominance,
    int seed = RANDOM_SEED
    );


inline constexpr std::string PRICING_TABU_SEARCH = "tabu_search";
std::vector<Route> full_pricing_problems_tabu_search(
    const DualSolution & solution,
    const Instance & instance,
    const std::vector<int> &vehicle_order,
    bool using_cyclic_pricing,
    int n_ressources_dominance,
    int max_iterations,
    int max_modifications,
    int seed = RANDOM_SEED
    );


inline constexpr std::string PRICING_PULSE_BASIC = "basic_pulse";
std::vector<Route> full_pricing_problems_basic_pulse(
    const DualSolution & solution,
    const Instance & instance,
    const std::vector<int> & vehicle_order,
    int delta = 10,
    int pool_size = 10
    );


inline constexpr std::string PRICING_PULSE_GROUPED = "grouped_pulse";
std::vector<Route> full_pricing_problems_grouped_pulse(
    const DualSolution & solution,
    const Instance & instance,
    const std::map<int, std::vector<int>> & vehicle_groups,
    int delta = 10,
    int pool_size = 10
    );

// Solve the pricing problems using the multithreaded PA, sequentially
inline constexpr std::string PRICING_PULSE_MULTITHREADED = "multi_pulse";
std::vector<Route> full_pricing_problems_multithreaded_pulse(
    const DualSolution & solution,
    const Instance & instance,
    const std::vector<int> & vehicle_order,
    int delta = 10,
    int pool_size = 10
    );


// Solve the pricing problems in groups using the multithreaded PA, sequentially
inline constexpr std::string PRICING_PULSE_GROUPED_MULTITHREADED = "mult_grouped_pa";
std::vector<Route> full_pricing_problems_grouped_pulse_multithreaded(
    const DualSolution & solution,
    const Instance & instance,
    const std::map<int, std::vector<int>> & vehicle_groups,
    int delta = 10,
    int pool_size = 10
    );