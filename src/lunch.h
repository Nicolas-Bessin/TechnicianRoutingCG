# pragma once

#include "../pathwyse/core/resources/resource.h"


class LunchBreak : public Resource<int> {
    public:
        LunchBreak();
        LunchBreak(int n_nodes);
        ~LunchBreak() = default;

        int extend(int current_value, int i, int j, bool direction) override;
        int join(int current_value_forward, int current_value_backward, int i, int j) override;
        int join(int current_value_forward, int current_value_backward, int node) override;
        bool isFeasible(int current_value, int current_node, double bounding, bool direction) override;

        // Setup the intervention : does this intervention need to respect the lunch break constraints ?
        void setConstrainedIntervention(int node, bool is_constrained);

    private:
        std::vector<bool> constrained_lunch_break;
};