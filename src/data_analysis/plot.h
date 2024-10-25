#pragma once

#include "instance/instance.h"
#include "routes/route.h"
#include <vector>

#include <matplot/matplot.h>


/*
    Plot the instance of the problem
    If given, plot the routes as well
*/
void plot_instance(
    const Instance& instance, 
    const std::vector<Route>& routes = std::vector<Route>()
);



/*
    Plot a list of values at given time points given in ms
    Convert the time points to seconds
    Optionnaly, remove the first N point and plot
    Use either normal or semilogy plot
*/
void plot_objective_values(
    matplot::axes_handle& ax,
    const std::vector<int>& time_points,
    const std::vector<double>& values,
    const std::string& name,
    bool use_semilogy = false,
    int N = 0
);

/*
    Plot a list of values at given time points given in ms
    Convert the time points to seconds
    Optionnaly, remove the first N point and plot
    Use either normal or semilogy plot
*/
void plot_objective_values(
    matplot::axes_handle& ax,
    const std::vector<int>& time_points,
    const std::vector<int>& values,
    const std::string& name,
    bool use_semilogy = false,
    int N = 0
);




