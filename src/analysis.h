#pragma once

#include "instance.h"
#include "solution.h"

#include <vector>

int count_covered_interventions(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

int count_used_vehicles(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

bool is_route_feasible(const Route& route, const Instance& instance);

double time_spent_travelling(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

double time_spent_working(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

double time_spent_waiting(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

// Count the number of interventions the vehicles used in the solution can perform
// (without considering the time windows, only the skills)
int count_coverable_interventions(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

int count_routes_with_duplicates(const std::vector<Route>& routes);

int count_used_routes_with_duplicates(const IntegerSolution& solution, const std::vector<Route>& routes);

double count_route_kilometres(const Route& route, const Instance& instance);

double count_kilometres_travelled(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

double compute_integer_objective(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

// Prints a route to the console
void print_route(const Route& route, const Instance& instance);

// Print the used routes in a solution
void print_used_routes(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

// Print the interventions not covered by the solution
void print_non_realised_interventions(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);