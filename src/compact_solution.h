#pragma once

#include "instance.h"

#include <vector>


/*
    Templated struct to store the solution of the compact model
*/
template <typename T>
struct CompactSolution{
    double objective_value;
    std::vector<std::vector<std::vector<T>>> x;
    std::vector<T> y;
    std::vector<int> u;

    // Default constructor
    CompactSolution() : objective_value(0.0), x(), y(), u() {}

    // Constructor with sizes
    CompactSolution(int n_nodes, int n_interventions, int n_vehicles) : objective_value(0.0), x(n_nodes, std::vector<std::vector<T>>(n_nodes, std::vector<T>(n_vehicles))), y(n_vehicles), u(n_interventions) {}
};


template <typename T>
double evaluate_compact_solution(const CompactSolution<T>& compact_solution, const Instance& instance) {
    double objective_value = 0.0;
    
    int n_nodes = instance.nodes.size();
    int n_vehicles = instance.number_vehicles;

    for (int v = 0; v < n_vehicles; v++) {
        objective_value -= compact_solution.y[v] * instance.vehicles[v].cost;
        for (int i = 0; i < n_nodes; i++) {
            for (int j = 0; j < n_nodes; j++) {
                const Node& node_i = instance.nodes[i];
                const Node& node_j = instance.nodes[j];
                int distance = instance.distance_matrix[i][j];
                double coef = instance.M * node_i.duration - instance.cost_per_km * distance;
                objective_value += compact_solution.x[i][j][v] * coef;
            }
        }
    }

    return objective_value;

}