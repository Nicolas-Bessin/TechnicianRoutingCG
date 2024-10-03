#pragma once

#include "instance/instance.h"
#include "master_problem/master.h"
#include "routes/route.h"
#include "master_problem/node.h"

#include "gurobi_c++.h"

#include <vector>


// - Creates a Gurobi model from an instance and a set of routes
// - Populates the vector of variables and the vectors of constraints
// @param instance : the instance of the problem
// @param routes : the vector of routes
// @param route_vars : the vector of variables associated with the routes
// @param intervention_ctrs : the vector of constraints associated with the interventions
// @param vehicle_ctrs : the vector of constraints associated with the vehicles
GRBModel create_model(
    const Instance& instance,
    const std::vector<Route>& routes,
    std::vector<GRBVar>& route_vars,
    std::vector<GRBVar>& postpone_vars,
    std::vector<GRBConstr>& intervention_ctrs,
    std::vector<GRBConstr>& vehicle_ctrs
);


// Adds a route to the model
// - Create the new variable and add it to the model & route_vars
// - Update the intervention constraints
// - Update the vehicle constraints
void add_route(
    GRBModel& model,
    const Route& route,
    const Instance& instance,
    std::vector<GRBVar>& route_vars,
    std::vector<GRBConstr>& intervention_ctrs,
    std::vector<GRBConstr>& vehicle_ctrs
);


// Solve the current version of the model
// Returns a status code
int solve_model(GRBModel& model);


// Extract the current solution as MasterSolution object
MasterSolution extract_solution(
    const GRBModel& model,
    const std::vector<GRBVar>& route_vars,
    const std::vector<GRBConstr>& intervention_ctrs,
    const std::vector<GRBConstr>& vehicle_ctrs
);


