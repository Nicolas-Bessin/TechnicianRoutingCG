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


// Computes the optimal clustering of the vehicles into clusters of 2 (at most one vehicle is alone in a cluster)
std::vector<std::vector<int>> optimal_clustering_by_2(std::vector<std::vector<int>> similarity_matrix);

// Compute an optimal partition of the vehicles into 2 clusters
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



// Basic regrouping of vehicles into groups that have the same depot
std::map<int, std::vector<int>> regroup_vehicles_by_depot(const std::vector<Vehicle>& vehicles);