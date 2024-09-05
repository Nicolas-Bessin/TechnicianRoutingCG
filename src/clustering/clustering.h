#pragma once

#include "instance/instance.h"

#include <vector>


// Computes the Hamming distance between every pair of vehicles
// Distance is defined by the "difference" in the interventions each vehicle can do
// Each intervention that can only be done by one of the vehicles contributes 1 to the distance
int hamming_distance(const Vehicle& vehicle1, const Vehicle& vehicle2);


// Computes the similarity matrix between vehicles 
// Based on the Hamming distance between vehicles
std::vector<std::vector<int>> compute_similarity_matrix(const std::vector<Vehicle>& vehicles);


// Computes the optimal 2-clustering of the vehicles (at most one vehicle is alone in a cluster)
std::vector<std::vector<int>> optimal_2_clustering(std::vector<std::vector<int>> similarity_matrix);