#pragma once

#include <vector>
#include <tuple>
#include <map>

// This represents a solution to the RMP dual problem
struct DualSolution {
    std::vector<double> alphas; // Intervention duals
    std::vector<double> betas; // Vehicle duals
    std::map<std::tuple<int, int, int>, double> upper_bound_duals;
    std::map<std::tuple<int, int, int>, double> lower_bound_duals;
};

// Define convex combination of two dual solutions
DualSolution operator+(const DualSolution &lhs, const DualSolution &rhs);

DualSolution operator*(double scalar, const DualSolution &rhs);

// Structure to represent a solution of the master problem
struct MasterSolution {
    // Is this solution feasible
    bool is_feasible;
    // Coefficients of the variables in the master problem
    std::vector<double> coefficients;
    // Dual solution
    DualSolution dual_solution;
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
        dual_solution({alphas, betas, upper_bound_duals, lower_bound_duals}),
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

