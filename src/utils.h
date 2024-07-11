#include "instance.h"

// Get a metric between two nodes based on a distance matrix (could be time or distance)
double metric(Node* node1, Node* node2, vector<vector<double>> metric_matrix){
    return metric_matrix[node1->node_id][node2->node_id];
};