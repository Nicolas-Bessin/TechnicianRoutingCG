#pragma once


#include "instance/instance.h"
#include "routes/route.h"
#include "master_problem/master.h"

#include <vector>
#include <random>
#include <array>

inline constexpr int RANDOM_SEED = -1;


inline constexpr std::string PRICING_PATHWYSE_BASIC = "basic_pathwyse";
std::vector<Route> full_pricing_problems_basic(
    const DualSolution & solution,
    const Instance & instance,
    const std::vector<int> & vehicle_order,
    bool use_maximisation_formulation,
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
    bool use_maximisation_formulation,
    bool using_cyclic_pricing,
    int n_ressources_dominance,
    int seed = RANDOM_SEED
);

inline constexpr std::string PRICING_CLUSTERING = "clustering";
std::vector<Route> full_pricing_problems_clustering(
    const DualSolution & solution,
    const Instance & instance,
    const std::vector<int> &vehicle_order,
    bool use_maximisation_formulation,
    bool using_cyclic_pricing,
    int n_ressources_dominance,
    int seed = RANDOM_SEED
    );


inline constexpr std::string PRICING_TABU_SEARCH = "tabu_search";
std::vector<Route> full_pricing_problems_tabu_search(
    const DualSolution & solution,
    const Instance & instance,
    const std::vector<int> &vehicle_order,
    bool use_maximisation_formulation,
    bool using_cyclic_pricing,
    int n_ressources_dominance,
    int max_iterations,
    int max_modifications,
    int seed = RANDOM_SEED
    );


inline constexpr std::string PRICING_PA_BASIC = "basic_pulse";
std::vector<Route> full_pricing_problems_basic_pulse(
    const DualSolution & solution,
    const Instance & instance,
    const std::vector<int> & vehicle_order,
    bool use_maximisation_formulation,
    int delta = 10,
    int pool_size = 10,
    bool verbose = false
    );


inline constexpr std::string PRICING_PA_GROUPED = "grouped_pulse";
std::vector<Route> full_pricing_problems_grouped_pulse(
    const DualSolution & solution,
    const Instance & instance,
    const std::map<int, std::vector<int>> & vehicle_groups,
    bool use_maximisation_formulation,
    int delta = 10,
    int pool_size = 10,
    bool verbose = false
    );

// Solve the pricing problems using the multithreaded PA, sequentially
inline constexpr std::string PRICING_MPA = "multi_pulse";
std::vector<Route> full_pricing_problems_multithreaded_pulse(
    const DualSolution & solution,
    const Instance & instance,
    const std::vector<int> & vehicle_order,
    bool use_maximisation_formulation,
    int delta = 10,
    int pool_size = 10,
    bool verbose = false
    );


// Solve the pricing problems in groups using the multithreaded PA, sequentially
inline constexpr std::string PRICING_MPA_GROUPED = "mult_grouped_pa";
std::vector<Route> full_pricing_problems_grouped_pulse_multithreaded(
    const DualSolution & solution,
    const Instance & instance,
    const std::map<int, std::vector<int>> & vehicle_groups,
    bool use_maximisation_formulation,
    int delta = 10,
    int pool_size = 10,
    bool verbose = false
    );


// Solve the grouped pricing problems in parallel, using the multithreaded PA
inline constexpr std::string PRICING_MPA_GROUPED_PAR = "grouped_par_par";
std::vector<Route> full_pricing_problems_grouped_pulse_par_par(
    const DualSolution & solution,
    const Instance & instance,
    const std::map<int, std::vector<int>> & vehicle_groups,
    bool use_maximisation_formulation,
    int delta = 10,
    int pool_size = 10,
    bool verbose = false
    );


// Define the set of PA derived algorithms
inline constexpr std::array<std::string, 5> PA_VARIATIONS = {
    PRICING_PA_BASIC,
    PRICING_PA_GROUPED,
    PRICING_MPA,
    PRICING_MPA_GROUPED,
    PRICING_MPA_GROUPED_PAR
};


inline constexpr std::string PRICING_PW_PA = "pathwyse+pulse";