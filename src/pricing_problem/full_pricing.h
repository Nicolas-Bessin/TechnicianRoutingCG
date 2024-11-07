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
    SolverMode solver_objective_mode = SolverMode::BIG_M_FORMULATION_OUTSOURCE,
    bool using_cyclic_pricing = false,
    int n_ressources_dominance = -1
);



inline constexpr std::string PRICING_DIVERSIFICATION = "diversification";
/*
    Generates a new set of route based on the diversitication algorithm presented in Dupin et al. (2021) 
*/
std::vector<Route> full_pricing_problems_diversification(
    const DualSolution & solution,
    const Instance & instance,
    const std::vector<int> & vehicle_order,
    SolverMode solver_objective_mode = SolverMode::BIG_M_FORMULATION_OUTSOURCE,
    bool using_cyclic_pricing = false,
    int n_ressources_dominance = -1,
    int seed = RANDOM_SEED
);

inline constexpr std::string PRICING_PA_BASIC = "basic_pulse";
std::vector<Route> full_pricing_problems_basic_pulse(
    const DualSolution & solution,
    const Instance & instance,
    const std::vector<int> & vehicle_order,
    SolverMode solver_objective_mode = SolverMode::BIG_M_FORMULATION_OUTSOURCE,
    int delta = 10,
    int pool_size = 10,
    bool verbose = false
    );


inline constexpr std::string PRICING_PA_GROUPED = "grouped_pulse";
std::vector<Route> full_pricing_problems_grouped_pulse(
    const DualSolution & solution,
    const Instance & instance,
    const std::map<int, std::vector<int>> & vehicle_groups,
    SolverMode solver_objective_mode = SolverMode::BIG_M_FORMULATION_OUTSOURCE,
    int delta = 10,
    int pool_size = 10,
    bool verbose = false
    );

// Define the set of PA derived algorithms
inline constexpr std::array<std::string, 5> PA_VARIATIONS = {
    PRICING_PA_BASIC,
    PRICING_PA_GROUPED,
};
