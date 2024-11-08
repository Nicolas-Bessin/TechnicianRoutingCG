#include "time_window_lunch.h"
#include "instance/constants.h"


CustomTimeWindow::CustomTimeWindow() {
    has_lunch_constraint.resize(0);
}

CustomTimeWindow::CustomTimeWindow(int n_nodes) {
    has_lunch_constraint.resize(n_nodes, false);
}


void CustomTimeWindow::setLunchConstraint(int node, bool has_constraint) {
    has_lunch_constraint.at(node) = has_constraint;
}

bool CustomTimeWindow::hasLunchConstraint(int node) const {
    return has_lunch_constraint.at(node);
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
    upper_bound = std::min(upper_bound, END_DAY);
}

int CustomTimeWindow::extend(int current_value, int i, int j, bool direction) {
    int current_time = current_value;

    if(direction) {
        current_time += data->getNodeCost(i) + data->getArcCost(i, j);
        // If we arrive too early, we wait until the time window opens
        if (current_time < node_lower_bound[j]) {
            current_time = node_lower_bound[j];
        }
        // If we the time of arrival leaves us no possibility of completing the intervention before lunch
        // And we could complete it after : we wait until the lunch break is over
        bool is_lunch_break = current_time + data->getNodeCost(j) > MID_DAY && current_time <= MID_DAY;
        if(has_lunch_constraint[j] && is_lunch_break) {
            current_time = MID_DAY;
        }
    }
    else {
        current_time += data->getArcCost(i, j);
        // If this departure time from i is too late, it means we wait after the time window closes
        if (current_time < upper_bound - node_upper_bound[i]) {
            current_time = upper_bound - node_upper_bound[i];
        }
        current_time += data->getNodeCost(i);
        // If this overlaps with the lunch break, we offset the time such that we do it entirely before the break
        bool is_lunch_break = current_time >= upper_bound - MID_DAY && current_time < upper_bound - MID_DAY + data->getNodeCost(i);
        if(has_lunch_constraint[i] && is_lunch_break) {
            current_time = upper_bound - MID_DAY + data->getNodeCost(i);
        }
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

    if (node_lower_bound[current_node] > node_upper_bound[current_node]) return false;

    if(current_node >= 0) {
        int feasible_value = 0;
        if(direction) {
            feasible_value = node_upper_bound[current_node];
        }
        else {
            feasible_value = upper_bound - (node_lower_bound[current_node]);
        }
        if(current_value > feasible_value) return false;
    }
    return true;
}

