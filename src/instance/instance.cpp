#include "instance.h"

#include "clustering/clustering.h"

#include <iostream>
#include <bits/stdc++.h>
#include <algorithm>

Vehicle vehicle_mask(const Vehicle& vehicle, const std::vector<int>& mask, bool mode){
    std::vector<int> new_interventions;
    std::map<int, int> new_reverse_map;
    for (int intervention : vehicle.interventions){
        if (mask[intervention] == mode){
            new_interventions.push_back(intervention);
            new_reverse_map[intervention] = new_interventions.size() - 1;
        }
    }
    return Vehicle{vehicle.id, vehicle.technicians, vehicle.skills, new_interventions, new_reverse_map, vehicle.depot, vehicle.capacities, vehicle.cost};
}


bool is_symmetric(const std::vector<std::vector<int>>& matrix) {
    // First, to be symetric, the matrix must be square
    if (matrix.size() != matrix[0].size()){
        return false;
    }
    for (int i = 0; i < matrix.size(); i++){
        for (int j = 0; j < matrix.size(); j++){
            if (matrix[i][j] != matrix[j][i]){
                return false;
            }
        }
    }
    return true;
}

int symmetry_gap(const std::vector<std::vector<int>>& matrix) {
    // First, to be symetric, the matrix must be square
    if (matrix.size() != matrix[0].size()){
        throw std::invalid_argument("Matrix is not square");
    }
    int biggest_diff = 0;
    for (int i = 0; i < matrix.size(); i++){
        for (int j = 0; j < matrix.size(); j++){
            biggest_diff = std::max(biggest_diff, std::abs(matrix[i][j] - matrix[j][i]));
        }
    }
    return biggest_diff;
}

bool can_do_intervention(const Node& intervention, const Vehicle& vehicle){
    // Check that the vehicle has the skills to do the intervention
    for (const auto &[skill, quantity] : intervention.required_skills){
        // If the skill is not in the vehicle, the vehicle cannot do the intervention
        if (vehicle.skills.find(skill) == vehicle.skills.end()){
            return false;
        }
        // If the vehicle does not have enough technicians with the skill, the vehicle cannot do the intervention
        if (vehicle.skills.at(skill) < quantity){
            return false;
        }
    }
    return true;
}


bool is_edge_feasible(int i, int j, const Instance& instance){
    const Node& intervention_i = instance.nodes[i];
    const Node& intervention_j = instance.nodes[j];

    // Check the time window : can j be done if we depart from i at the last possible moment ?
    int arrival_time = intervention_i.start_window + intervention_i.duration + instance.time_matrix[i][j];

    return arrival_time + intervention_j.duration <= intervention_j.end_window;
}

void check_triangular_inequality(const Instance& instance) {
    using std::cout, std::endl;
    // First, we check that the distance matrix verifies the triangular inequality
    for (int i = 0; i < instance.number_interventions; i++){
        for (int j = 0; j < instance.number_interventions; j++){
            for (int k = 0; k < instance.number_interventions; k++){
                if (instance.distance_matrix[i][j] > instance.distance_matrix[i][k] + instance.distance_matrix[k][j]){
                    cout << "Distance matrix does not verify the triangular inequality" << endl;
                    cout << "d(" << i << ", " << j << ") = " << instance.distance_matrix[i][j] << endl;
                    cout << "d(" << i << ", " << k << ") + d(" << k << ", " << j << ") = " << instance.distance_matrix[i][k] + instance.distance_matrix[k][j] << endl;
                }
            }
        }
    }

    // Then, we check that the time matrix verifies the triangular inequality
    for (int i = 0; i < instance.number_interventions; i++){
        for (int j = 0; j < instance.number_interventions; j++){
            for (int k = 0; k < instance.number_interventions; k++){
                if (instance.time_matrix[i][j] > instance.time_matrix[i][k] + instance.time_matrix[k][j] + instance.nodes[k].duration){
                    cout << "Time matrix does not verify the triangular inequality" << endl;
                    cout << "t(" << i << ", " << j << ") = " << instance.time_matrix[i][j] << endl;
                    cout << "t(" << i << ", " << k << ") + d(" << k << ") + t(" << k << ", " << j << ") = " << instance.time_matrix[i][k] + instance.nodes[k].duration + instance.time_matrix[k][j] << endl;
                }
            }
        }
    }
}


double compute_M_naive(const Instance& instance) {
    using std::__gcd;
    using std::vector;
    // Finally, compute the big M for the objective function
    // M is computed using : M = (END_DAY - min(durations .> 0)) * maxspeed * cost_per_km / gcd(durations)
    // We first put all the durations in a vector
    vector<int> durations = vector<int>();
    // Enumerate through the nodes
    for (int i = 0; i < instance.number_interventions; i++){
        // Only keep the durations that are greater than 0
        if (instance.nodes[i].duration > 0){
            durations.push_back(instance.nodes[i].duration);
        }
    }
    int min_duration = *min_element(durations.begin(), durations.end());
    // We now compute the gcd of the durations
    int gcd_durations = durations[0];
    for (int i = 1; i < durations.size(); i++){
        gcd_durations = __gcd(gcd_durations, durations[i]);
    }
    // We finally compute the maximum speed using the ditance and time matrices
    // Only among the pairs of node that we parsed previously
    double max_speed = 0;
    std::pair<int, int> max_speed_pair = std::make_pair(0, 0);
    for (int i = 0; i < instance.nodes.size(); i++){
        for (int j = 0; j < instance.nodes.size(); j++){
            if (instance.time_matrix[i][j] > 0){
                double speed = instance.distance_matrix[i][j] / (double) instance.time_matrix[i][j];
                if (speed > max_speed){
                    max_speed = speed;
                    max_speed_pair = std::make_pair(i, j);
                }
            }
        }
    }

    double M = (END_DAY - min_duration) * max_speed * instance.cost_per_km / gcd_durations;

    return M;
}

double compute_M_perV(const Instance& instance) {
    using std::__gcd;
    using std::vector;

    // Finally, compute the big M for the objective function
    // M is computed using : M = (END_DAY - min(durations .> 0)) * maxspeed * cost_per_km / gcd(durations)
    // We first put all the durations in a vector
    vector<int> durations = vector<int>();
    // Enumerate through the nodes
    for (int i = 0; i < instance.number_interventions; i++){
        // Only keep the durations that are greater than 0
        if (instance.nodes[i].duration > 0){
            durations.push_back(instance.nodes[i].duration);
        }
    }
    int min_duration = *min_element(durations.begin(), durations.end());
    // We now compute the gcd of the durations
    int gcd_durations = durations[0];
    for (int i = 1; i < durations.size(); i++){
        gcd_durations = __gcd(gcd_durations, durations[i]);
    }
    // We finally compute the maximum speed using the ditance and time matrices
    // Only among the pairs of node that we parsed previously
    double max_speed = 0;
    std::pair<int, int> max_speed_pair = std::make_pair(0, 0);
    for (int i = 0; i < instance.nodes.size(); i++){
        for (int j = 0; j < instance.nodes.size(); j++){
            if (instance.time_matrix[i][j] > 0){
                double speed = instance.distance_matrix[i][j] / (double) instance.time_matrix[i][j];
                if (speed > max_speed){
                    max_speed = speed;
                    max_speed_pair = std::make_pair(i, j);
                }
            }
        }
    }
    // Min cost per vehicle
    double min_vehicle_fixed_cost = std::numeric_limits<double>::infinity();
    for (const Vehicle& vehicle : instance.vehicles){
        min_vehicle_fixed_cost = std::min(min_vehicle_fixed_cost, vehicle.cost);
    }
    double max_fixed_expect_one = 0;
    for (const Vehicle& vehicle : instance.vehicles){
        max_fixed_expect_one += vehicle.cost;
    }
    max_fixed_expect_one -= min_vehicle_fixed_cost;

    // We finally compute the big M
    double M = ( (instance.number_vehicles * (END_DAY - min_duration) * max_speed * instance.cost_per_km)
        + max_fixed_expect_one ) / gcd_durations;

    return M;
}