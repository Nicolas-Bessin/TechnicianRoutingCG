#pragma once

#include "instance.h"
#include "route.h"


struct GeneticSolution {
    std::vector<Route> routes;
    double objective;
};