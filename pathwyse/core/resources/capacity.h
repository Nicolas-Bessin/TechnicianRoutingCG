#ifndef CAPACITY_H
#define CAPACITY_H

#include "resource.h"

class Capacity: public Resource<int> {

public:
    Capacity();
    ~Capacity() = default;

    int extend(int current_value, int i, int j, bool direction = true) override;
    int join(int current_value_forward, int current_value_backward, int i, int j) override;
    int join(int current_value_forward, int current_value_backward, int node) override;
    bool isFeasible(int current_value, int current_node = -1, double bounding = 1.0, bool direction = true) override;
};

#endif