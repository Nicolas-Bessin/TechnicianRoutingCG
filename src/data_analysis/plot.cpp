#include "plot.h"

#include <matplot/matplot.h>

#include <random>

#include "data_analysis/analysis.h"

void plot_instance(
    const Instance& instance,
    const std::vector<Route>& routes
    ) {
    using namespace matplot;
    // Create the graph
    auto f = figure(false);
    f->width(1200);
    f->height(800);
    f->title("Instance");

    // Plot the routes
    hold(on);

    for (const Route & route : routes){
        // Fix the color of the route
        std::random_device rd;
        std::mt19937 gen(rd());
        float red = std::uniform_real_distribution<float>(0, 1)(gen);
        float green = std::uniform_real_distribution<float>(0, 1)(gen);
        float blue = std::uniform_real_distribution<float>(0, 1)(gen);
        // std::cout << "Color : " << red << " " << green << " " << blue << std::endl;
        // Build the lon and lat vectors along the route
        std::vector<double> lon_route;
        std::vector<double> lat_route;
        for (int id : route.id_sequence){
            lon_route.push_back(instance.nodes[id].position.first);
            lat_route.push_back(instance.nodes[id].position.second);
        }
        int route_length = count_route_kilometres(route, instance);
        plot(lon_route, lat_route)
            ->color({red, green, blue})
            .marker_color({0.f, 0.f, 1.f})
            .display_name("v" + std::to_string(route.vehicle_id) + " - " + std::to_string(route_length) + " km");
    }

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

    scatter(lon_warehouses, lat_warehouses)
        ->marker_color({1.f, 0.f, 0.f})
        .marker_style(line_spec::marker_style::diamond)
        .display_name("Warehouses");

    scatter(lon_interventions, lat_interventions)
        ->marker_color({0.f, 0.f, 1.f})
        .display_name("Interventions");

    // Display the legend
    ::matplot::legend(off);
    // Show the graph
    hold(off);

    show();

}