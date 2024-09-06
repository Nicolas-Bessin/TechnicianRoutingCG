#include "clustering.h"

#include <map>
#include <vector>
#include <algorithm>
#include <random>

#include "gurobi_c++.h"


int hamming_distance(const Vehicle& vehicle1, const Vehicle& vehicle2){
    using std::map;

    map<int, int> interventions;
    // Store the interventions of each vehicle in a map
    for (int intervention : vehicle1.interventions){
        interventions[intervention] += 1;
    }
    for (int intervention : vehicle2.interventions){
        interventions[intervention] += 1;
    }
    // Those that have a value of 1 are only in one of the vehicles - they contribute 1 to the distance
    int distance = 0;
    for (auto [intervention, count] : interventions){
        if (count == 1){
            distance += 1;
        }
    }

    return distance;
}


std::vector<std::vector<int>> compute_similarity_matrix(const std::vector<Vehicle>& vehicles){
    int n = vehicles.size();
    std::vector<std::vector<int>> matrix(n, std::vector<int>(n, 0));
    for (int i = 0; i < n; i++){
        for (int j = i + 1; j < n; j++){
            matrix[i][j] = hamming_distance(vehicles[i], vehicles[j]);
            matrix[j][i] = matrix[i][j];
        }
    }
    return matrix;
}


std::vector<std::vector<int>> optimal_2_clustering(std::vector<std::vector<int>> similarity_matrix){

    using std::vector;
    int n = similarity_matrix.size();

    // Create a model
    GRBEnv env = GRBEnv(true);
    env.set(GRB_IntParam_OutputFlag, 0);
    env.start();
    GRBModel model = GRBModel(env);

    // Create variables
    vector<vector<GRBVar>> x(n, vector<GRBVar>(n));

    for (int i = 0; i < n; i++){
        for (int j = i+1; j < n; j++){
            x[i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
        }
    }

    // Add the constraints : At most 2 vehicles per cluster
    for (int i = 0; i < n; i++){
        GRBLinExpr expr = 0;
        for (int j = 0; j < i; j++){
            expr += x[j][i];
        }
        for (int j = i+1; j < n; j++){
            expr += x[i][j];
        }
        model.addConstr(expr <= 1);
    }

    // At most one cluster with only one vehicle
    GRBLinExpr expr = 0;
    for (int i = 0; i < n; i++){
        expr += 1;
        for (int j = 0; j < i; j++){
            expr -= x[j][i];
        }
        for (int j = i+1; j < n; j++){
            expr -= x[i][j];
        }
    }
    model.addConstr(expr <= 1);

    // Objective function
    GRBLinExpr obj = 0;
    for (int i = 0; i < n; i++){
        for (int j = i+1; j < n; j++){
            obj += similarity_matrix[i][j] * x[i][j];
        }
    }

    model.setObjective(obj, GRB_MINIMIZE);

    // Optimize the model
    model.optimize();

    // Retrieve the solution
    vector<vector<int>> clusters;

    // If x[i][j] = 1, then i and j are in the same cluster
    for (int i = 0; i < n; i++){
        for (int j = i+1; j < n; j++){
            if (x[i][j].get(GRB_DoubleAttr_X) > 0.5){
                clusters.push_back({i, j});
            }
        }
    }

    return clusters;
}



std::vector<std::vector<int>> greedy_neighbor(
    const std::vector<std::vector<int>>& similarity_matrix, 
    const std::vector<std::vector<int>>& initial_clusters,
    int seed 
    ){

    using std::vector, std::map;

    int n = similarity_matrix.size();
    // Copy the initial clusters
    vector<vector<int>> clusters = initial_clusters;
    
    // Create a map to store the correspondance between vehicles and their cluster
    map<int, int> vehicle_to_cluster_id;

    for (int i = 0; i < clusters.size(); i++){
        for (int vehicle : clusters[i]){
            vehicle_to_cluster_id[vehicle] = i;
        }
    }

    // Pick a random vehicle and swap it with it's closest neighbor that is not in the same cluster
    std::mt19937 gen(seed);
    std::uniform_int_distribution<int> dis(0, n-1);

    int vehicle = dis(gen);
    int cluster = vehicle_to_cluster_id[vehicle];

    // Find the closest neighbor that is not in the same cluster
    int closest_neighbor = -1;
    int min_distance = INT32_MAX;
    for (int i = 0; i < n; i++){
        if (i == vehicle){
            continue;
        }
        if (vehicle_to_cluster_id[i] == cluster){
            continue;
        }
        if (similarity_matrix[vehicle][i] < min_distance){
            min_distance = similarity_matrix[vehicle][i];
            closest_neighbor = i;
        }
    }

    // Swap the vehicle and the closest neighbor
    // Find the indexes of both vehicles in their respective clusters
    auto it1 = std::find(clusters[cluster].begin(), clusters[cluster].end(), vehicle);
    int cluster_neighbor = vehicle_to_cluster_id[closest_neighbor];
    auto it2 = std::find(clusters[cluster_neighbor].begin(), clusters[cluster_neighbor].end(), closest_neighbor);
    // Swap the vehicles
    *it1 = closest_neighbor;
    *it2 = vehicle;

    return clusters;
}


int compute_clustering_cost(
    const std::vector<std::vector<int>>& clusters,
    const std::vector<std::vector<int>>& similarity_matrix
){
    int cost = 0;
    for (const auto& cluster : clusters){
        for (int i = 0; i < cluster.size(); i++){
            for (int j = i+1; j < cluster.size(); j++){
                cost += similarity_matrix[cluster[i]][cluster[j]];
            }
        }
    }
    return cost;
}