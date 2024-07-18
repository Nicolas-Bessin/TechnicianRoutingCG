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