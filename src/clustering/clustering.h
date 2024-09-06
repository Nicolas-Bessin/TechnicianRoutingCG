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


// Greedily compute a good neighbor of a given cluster repartition
std::vector<std::vector<int>> greedy_neighbor(
    const std::vector<std::vector<int>>& similarity_matrix, 
    const std::vector<std::vector<int>>& clusters,
    int seed = 0
);


// Compute the cost of a clustering
int compute_clustering_cost(
    const std::vector<std::vector<int>>& clusters,
    const std::vector<std::vector<int>>& similarity_matrix
);
