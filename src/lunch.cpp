#include "lunch.h"
#include "constants.h"


LunchBreak::LunchBreak() {
    constrained_lunch_break.resize(0);
}

LunchBreak::LunchBreak(int n_nodes) {
    constrained_lunch_break.resize(n_nodes, false);
}

void LunchBreak::setConstrainedIntervention(int node, bool is_constrained) {
    constrained_lunch_break.at(node) = is_constrained;
}

int LunchBreak::extend(int current_value, int i, int j, bool direction) {
    int current_time = current_value + data->getArcCost(i, j);

    if(direction) {
        current_time += data->getNodeCost(i);
        current_time = std::max(current_time, node_lower_bound[j]); //fw: arrival time at j
    }
    else {
        current_time += data->getNodeCost(j);
        current_time = std::max(current_time, upper_bound - (node_upper_bound[i] + data->getNodeCost(i)));    //bw: time between departure from j to arrival at destination
    }

    return current_time;
}

int LunchBreak::join(int current_value_forward, int current_value_backward, int i, int j){
    return current_value_forward + data->getNodeCost(i) + data->getArcCost(i, j) + data->getNodeCost(j) + current_value_backward;
}

int LunchBreak::join(int current_value_forward, int current_value_backward, int node){
    return current_value_forward + current_value_backward - data->getNodeCost(node);
}

bool LunchBreak::isFeasible(int current_value, int current_node, double bounding, bool direction) {
    bool need_to_respect_lunch_break = constrained_lunch_break[current_node];
    if (!need_to_respect_lunch_break) {
        return true;
    }
    int duration = data->getNodeCost(current_node);
    // Check wether the intervention respects the lunch break
    bool is_respected = current_value + duration <= MID_DAY || current_value >= MID_DAY;
    return is_respected;
}