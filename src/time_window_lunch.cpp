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


//Computes the upperbound.
//Represents the overall resource availability, i.e. the maximum feasible arrival time at destination.
void CustomTimeWindow::init(int origin, int destination) {
    upper_bound = 0;

    for(int i = 0; i < node_upper_bound.size(); i++)
        if(i != destination)
            upper_bound = std::max(upper_bound, node_upper_bound[i] + data->getNodeCost(i) + data->getArcCost(i, destination));

    // The upper bound also has to respect the time window of the destination node
    upper_bound = std::min(upper_bound, node_upper_bound[destination]);
}

int CustomTimeWindow::extend(int current_value, int i, int j, bool direction) {
    int current_time = current_value;

    if(direction) {
        current_time += data->getNodeCost(i) + data->getArcCost(i, j);
        // If we arrive too early, we wait until the time window opens
        current_time = std::max(current_time, node_lower_bound[j]); //fw: arrival time at j
        // If we the time of arrival leaves us no possibility of completing the intervention before lunch
        // And we could complete it after : we wait until the lunch break is over
        bool is_lunch_break = current_time + data->getNodeCost(j) > MID_DAY && current_time < MID_DAY;
        if(has_lunch_constraint[j] && is_lunch_break) {
            current_time = MID_DAY;
        }
    }
    else {
        current_time += data->getNodeCost(j) + data->getArcCost(i, j);
        current_time = std::max(current_time, upper_bound - (node_upper_bound[i] + data->getNodeCost(i)));    //bw: time between departure from j to arrival at destination
    }

    return current_time;
}

int CustomTimeWindow::join(int current_value_forward, int current_value_backward, int i, int j){
    return current_value_forward + data->getNodeCost(i) + data->getArcCost(i, j) + data->getNodeCost(j) + current_value_backward;
}

int CustomTimeWindow::join(int current_value_forward, int current_value_backward, int node){
    return current_value_forward + current_value_backward - data->getNodeCost(node);
}

bool CustomTimeWindow::isFeasible(int current_value, int current_node, double bounding, bool direction) {
    if(current_value > upper_bound*bounding) return false;

    if(current_node >= 0) {
        int feasible_value = direction ? node_upper_bound[current_node] : upper_bound - (node_lower_bound[current_node] + data->getNodeCost(current_node));
        if(current_value >= feasible_value) return false;
    }
    // We we reached this point, we know know the time window is respected
    // Let's check if the lunch constraint is respected
    if(!has_lunch_constraint[current_node]) return true;

    bool is_respected = current_value + data->getNodeCost(current_node) <= MID_DAY || current_value > MID_DAY;

    return is_respected;
}

