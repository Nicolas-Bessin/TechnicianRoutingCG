#pragma once

#include "instance/instance.h"
#include "routes/route.h"
#include <vector>


/*
    Plot the instance of the problem
    If given, plot the routes as well
*/
void plot_instance(
    const Instance& instance, 
    const std::vector<Route>& routes = std::vector<Route>()
);


