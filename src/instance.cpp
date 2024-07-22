#include "instance.h"

// Get a metric between two nodes based on a distance matrix (could be time or distance)
int metric(const Node& node1, const Node& node2, std::vector<std::vector<int>> metric_matrix){
    return metric_matrix.at(node1.node_id).at(node2.node_id);
};