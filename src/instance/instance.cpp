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

Instance cut_instance(const Instance& instance, const std::vector<int>& mask) {
    // We only keep the interventions that are not masked
    // We need to remove the vehicles that cannot do any intervention
    // We need to update the vehicles
    // And the time and distance matrices

    // First, update the interventions themselves
    std::vector<Node> new_nodes;
    std::map<int, int> old_to_new;
    std::map<int, int> new_to_old;

    for (int i = 0; i < instance.number_interventions; i++){
        if (mask[i] == 1){
            new_nodes.push_back(instance.nodes[i]);
            old_to_new[i] = new_nodes.size() - 1;
            new_to_old[new_nodes.size() - 1] = i;
        }
    }
    // Add the warehouses
    for (int i = instance.number_interventions; i < instance.nodes.size(); i++){
        new_nodes.push_back(instance.nodes[i]);
        old_to_new[i] = new_nodes.size() - 1;
        new_to_old[new_nodes.size() - 1] = i;
    }

    // Update the vehicles' interventions
    std::vector<Vehicle> new_vehicles;
    for (const Vehicle& vehicle : instance.vehicles){
        // Update the interventions & reverse map
        std::vector<int> new_interventions;
        std::map<int, int> new_reverse_map;
        for (int intervention : vehicle.interventions){
            if (mask[intervention] == 1){
                new_interventions.push_back(old_to_new[intervention]);
                new_reverse_map[old_to_new[intervention]] = new_interventions.size() - 1;
            }
        }
        // If the vehicle can do at least one intervention, we keep it
        Vehicle new_vehicle = Vehicle{
            vehicle.id, 
            vehicle.technicians, 
            vehicle.skills, 
            new_interventions, 
            new_reverse_map, 
            old_to_new[vehicle.depot],
            vehicle.capacities, 
            vehicle.cost};
        if (new_vehicle.interventions.size() > 0){
            new_vehicle.id = new_vehicles.size();
            new_vehicles.push_back(new_vehicle);
        }
    }

    // Update the time and distance matrices
    std::vector<std::vector<int>> new_time_matrix(new_nodes.size(), std::vector<int>(new_nodes.size(), 0));
    std::vector<std::vector<int>> new_distance_matrix(new_nodes.size(), std::vector<int>(new_nodes.size(), 0));
    for (int i = 0; i < new_nodes.size(); i++){
        for (int j = 0; j < new_nodes.size(); j++){
            new_time_matrix[i][j] = instance.time_matrix[new_to_old[i]][new_to_old[j]];
            new_distance_matrix[i][j] = instance.distance_matrix[new_to_old[i]][new_to_old[j]];
        }
    }

    // Re-compute the similarity matrix
    std::vector<std::vector<int>> new_similarity_matrix = compute_similarity_matrix(new_vehicles);

    return Instance{
        instance.name,
        new_nodes.size() - instance.number_warehouses,
        instance.number_warehouses,
        new_vehicles.size(),
        instance.cost_per_km,
        instance.technician_cost,
        instance.M,
        new_nodes,
        new_vehicles,
        instance.capacities_labels,
        new_time_matrix,
        new_distance_matrix,
        new_similarity_matrix
    };
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
    double min_vehicle_fixed_cost = 0;
    for (const Vehicle& vehicle : instance.vehicles){
        min_vehicle_fixed_cost = std::min(min_vehicle_fixed_cost, vehicle.cost);
    }
    double max_fixed_expect_one = 0;
    for (const Vehicle& vehicle : instance.vehicles){
        max_fixed_expect_one += vehicle.cost;
    }
    max_fixed_expect_one -= min_vehicle_fixed_cost;

    // We finally compute the big M
    double M = ( instance.number_vehicles * (END_DAY - min_duration) * max_speed * instance.cost_per_km
        + max_fixed_expect_one ) / gcd_durations;

    return M;
}