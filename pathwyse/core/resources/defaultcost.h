#ifndef DEFAULTCOST_H
#define DEFAULTCOST_H

#include "resource.h"

class DefaultCost: public Resource<double> {

public:
    DefaultCost();
    DefaultCost(double init_value);
    ~DefaultCost() = default;

    double extend(double current_value, int i, int j, bool direction) override;
    double join(double current_value_forward, double current_value_backward, int i, int j) override;
    double join(double current_value_forward, double current_value_backward, int node) override;
    bool isFeasible(double current_value, int current_node, double bounding, bool direction) override;
};


#endif