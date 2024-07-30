#pragma once

#include "instance.h"
#include "solution.h"

#include <vector>


struct CompactSolution{
    double objective_value;
    std::vector<std::vector<std::vector<int>>> x;
    std::vector<int> y;
    std::vector<int> u;

    // Default constructor
    CompactSolution() : objective_value(0.0), x(), y(), u() {}

    // Constructor with sizes
    CompactSolution(int n_nodes, int n_interventions, int n_vehicles) : objective_value(0.0), x(n_nodes, std::vector<std::vector<int>>(n_nodes, std::vector<int>(n_vehicles))), y(n_vehicles), u(n_interventions) {}
};

/*
    Solves the compact model for the problem
    @param instance: the instance of the problem
    @param time_limit: the time limit for the solver
    @param imposed_routings: a vector of pairs (vehicle_id, intervention_id) the vehicle to go thorugh the intervention (default empty)
    @return a CompactSolution struct with the values of the variables
*/
CompactSolution compact_solver(const Instance& instance, int time_limit = 60, std::vector<std::pair<int, int>> imposed_routings = std::vector<std::pair<int, int>>() );

std::vector<Route> compact_solution_to_routes(const Instance& instance, const CompactSolution& compact_solution);