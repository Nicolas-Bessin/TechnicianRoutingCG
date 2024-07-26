#include "instance.h"

// Get a metric between two nodes based on a distance matrix (could be time or distance)
int metric(const Node& node1, const Node& node2, std::vector<std::vector<int>> metric_matrix){
    return metric_matrix.at(node1.node_id).at(node2.node_id);
};


Vehicle vehicle_mask(const Vehicle& vehicle, const std::vector<int>& mask){
    std::vector<int> new_interventions;
    for (int intervention : vehicle.interventions){
        if (mask[intervention] == 0){
            new_interventions.push_back(intervention);
        }
    }
    return Vehicle(vehicle.id, vehicle.technicians, vehicle.skills, new_interventions, vehicle.depot, vehicle.capacities, vehicle.cost);
}