#include "time_window_lunch.h"
#include "constants.h"


CustomTimeWindow::CustomTimeWindow() {
    has_lunch_constraint.resize(0);
}

CustomTimeWindow::CustomTimeWindow(int n_nodes) {
    has_lunch_constraint.resize(n_nodes, false);
}


void CustomTimeWindow::setLunchConstraint(int node, bool has_constraint) {
    has_lunch_constraint.at(node) = has_constraint;
}


int CustomTimeWindow::extend(int current_value, int i, int j, bool direction) {
    // Extend the time window, either forward from i to j or backward from j to i
    int current_time = current_value;
    // If forward, add the cost of node i.
    if(direction) {
        current_time += data->getNodeCost(i) + data->getArcCost(i, j);
        // If we arrive early at j, we wait until the time window opens
        current_time = std::max(current_time, node_lower_bound[j]);
    } else {
        // If backward, add the cost of node j
        current_time += data->getNodeCost(j) + data->getArcCost(i, j);
        // If we might have had to wait at i, we wait until the time window opens
        current_time = std::min(current_time, node_upper_bound[i]);
    }

    return current_time;
}


int CustomTimeWindow::join(int current_value_forward, int current_value_backward, int i, int j) {
    // Join the forward and backward time windows
    return current_value_forward + data->getNodeCost(i) + data->getArcCost(i, j) + data->getNodeCost(j) + current_value_backward;
}

int CustomTimeWindow::join(int current_value_forward, int current_value_backward, int node) {
    // Join the forward and backward time windows
    return current_value_forward + current_value_backward - data->getNodeCost(node);
}

bool CustomTimeWindow::isFeasible(int current_value, int current_node, double bounding, bool direction) {
    // First step is checking wether the time window is respected
    int duration = data->getNodeCost(current_node);
    bool is_tw = current_value >= node_lower_bound[current_node] && current_value <= node_upper_bound[current_node];
    // If the time window is not respected, we return false
    if (!is_tw) {
        return false;
    }
    // If the time window is respected, we check wether the lunch break is respected
    bool need_to_respect_lunch_break = has_lunch_constraint[current_node];
    if (!need_to_respect_lunch_break) {
        return true;
    }
    // If the lunch break needs to be respected, we check wether the intervention respects the lunch break
    // Check wether the intervention respects the lunch break
    bool is_respected = current_value + duration <= MID_DAY || current_value >= MID_DAY;

    return is_respected;
}


