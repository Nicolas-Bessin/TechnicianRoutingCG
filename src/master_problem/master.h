#pragma once

#include <vector>
#include <tuple>
#include <map>

// Structure to represent a solution of the master problem
struct MasterSolution {
    // Is this solution feasible
    bool is_feasible;
    // Coefficients of the variables in the master problem
    std::vector<double> coefficients;
    // Dual values associated with the interventions
    std::vector<double> alphas;
    // Dual values associated with the vehicles
    std::vector<double> betas;
    // Dual values associated with the upper bound cuts
    std::map<std::tuple<int, int, int>, double> upper_bound_duals;
    // Dual values associated with the lower bound cuts
    std::map<std::tuple<int, int, int>, double> lower_bound_duals;
    // Objective value of the master problem
    double objective_value;
    // Empty constructor
    MasterSolution() : is_feasible(false) {}
    // Constructor
    MasterSolution(
        std::vector<double> coefficients,
        std::vector<double> alphas,
        std::vector<double> betas,
        std::map<std::tuple<int, int, int>, double> upper_bound_duals,
        std::map<std::tuple<int, int, int>, double> lower_bound_duals,
        double objective_value
    ) : 
        coefficients(coefficients),
        alphas(alphas),
        betas(betas),
        upper_bound_duals(upper_bound_duals),
        lower_bound_duals(lower_bound_duals),
        objective_value(objective_value),
        is_feasible(true)
    {}
};

// Solution to the integer version of the problem
struct IntegerSolution {
    std::vector<int> coefficients;
    double objective_value;
    // Is this solution feasible
    bool is_feasible;
    // Empty constructor
    IntegerSolution() : is_feasible(false), objective_value(-1.) {}
    // Constructor
    IntegerSolution(std::vector<int> coefficients, double objective_value) : 
        coefficients(coefficients), objective_value(objective_value), is_feasible(true) {}
};

