#include "plot.h"

#include <matplot/matplot.h>

void plot_instance(const Instance& instance) {
    using namespace matplot;
    // Create the graph
    auto f = figure(true);
    f->width(1200);
    f->height(800);
    f->title("Instance");
    // Plot the nodes
    std::vector<double> lon_warehouses;
    std::vector<double> lat_warehouses;
    std::vector<double> lon_interventions;
    std::vector<double> lat_interventions;
    for (const Node& node : instance.nodes){
        if (node.is_intervention){
            lon_interventions.push_back(node.position.first);
            lat_interventions.push_back(node.position.second);
        } else {
            lon_warehouses.push_back(node.position.first);
            lat_warehouses.push_back(node.position.second);
        }
    }
    auto w = scatter(lon_warehouses, lat_warehouses);
    w->marker_color({1.f, 0.f, 0.f});
    w->marker_style(line_spec::marker_style::diamond);

    hold(on);
    auto i = scatter(lon_interventions, lat_interventions);
    i->marker_color({0.f, 0.f, 1.f});
    hold(off);
    // Show the graph
    show();
}