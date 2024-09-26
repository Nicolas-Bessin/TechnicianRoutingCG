#pragma once

#include "../../pathwyse/core/resources/resource.h"


/*
    At each node, we check if the time window is respected :
    The performed check is : 
    ```lb <= current_time && current_time + duration <= ub```
    where lb and ub are the lower and upper bounds of the time window, and duration is the time spent at the node.
*/
class CustomTimeWindow : public Resource<int> {
public:
    CustomTimeWindow();
    // Custom constructor : initialize the has_lunch_constraint vector
    CustomTimeWindow(int n_nodes);
    // Destructor
    ~CustomTimeWindow() = default;


    void init(int origin, int destination) override;
    int extend(int current_value, int i, int j, bool direction) override;
    int join(int current_value_forward, int current_value_backward, int i, int j) override;
    int join(int current_value_forward, int current_value_backward, int node) override;
    bool isFeasible(int current_value, int current_node, double bounding = 1.0, bool direction = true) override;

    void setLunchConstraint(int node, bool has_constraint);
private:
    std::vector<bool> has_lunch_constraint;

};