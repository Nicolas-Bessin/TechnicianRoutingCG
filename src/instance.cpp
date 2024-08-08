#include "instance.h"

#include <iostream>

Vehicle vehicle_mask(const Vehicle& vehicle, const std::vector<int>& mask){
    std::vector<int> new_interventions;
    std::map<int, int> new_reverse_map;
    for (int intervention : vehicle.interventions){
        if (mask[intervention] == 0){
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