#include "route_optimizer.h"

#include "analysis.h"

#include <vector>
#include "gurobi_c++.h"

Route optimize_route(const Route& route, const Instance& instance) {
    using namespace std;

    // Get the vehicle that will perform the route
    // Only keep the interventions that are in the route
    const Vehicle vehicle = vehicle_mask(instance.vehicles[route.vehicle_id], route.is_in_route, KEEP_COVERED);

    int n_nodes = vehicle.interventions.size() + 1;
    int n_intervention = vehicle.interventions.size();

    // Create the model
    GRBEnv env = GRBEnv();
    env.set(GRB_IntParam_OutputFlag, 0);
    GRBModel model = GRBModel(env);

    // Create the variables y_ij (binary variables indicating if the vehicle goes from node i to node j), the last node is the warehouse
    vector<vector<GRBVar>> y(n_nodes, vector<GRBVar>(n_nodes));
    for (int i = 0; i < n_nodes; i++) {
        for (int j = 0; j < n_nodes; j++) {
            y[i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
        }
    }
    // Create the time variables u_i (time at which the vehicle arrives at node i)
    vector<GRBVar> u(n_intervention);
    for (int i = 0; i < n_intervention; i++) {
        u[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
    }
    // Variables z_i - is the intervention i done in the afternoon ?
    vector<GRBVar> z(n_intervention);
    for (int i = 0; i < n_intervention; i++) {
        z[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
    }

    // Add the constraints
    // Routing constraint : flow conservation - and going through each intervention once
    // We get a single tour thanks to the time constraints later
    for (int i = 0; i < n_nodes; i++) {
        GRBLinExpr expr_in = 0;
        GRBLinExpr expr_out = 0;
        for (int j = 0; j < n_nodes; j++) {
            expr_in += y[j][i];
            expr_out += y[i][j];
        }
        model.addConstr(expr_in == 1);
        model.addConstr(expr_out == 1);
        model.addConstr(y[i][i] == 0);
    }
    // Time constraints
    // Time window constraints
    for (int i = 0; i < n_intervention; i++) {
        const Node& intervention = instance.nodes[vehicle.interventions[i]];
        model.addConstr(u[i] >= intervention.start_window);
        model.addConstr(u[i] + intervention.duration <= intervention.end_window);
    }
    // Consecutive interventions constraints
    for (int i = 0; i < n_intervention; i++) {
        const Node & intervention = instance.nodes[vehicle.interventions[i]];
        for (int j = 0; j < n_intervention; j++) {
            GRBLinExpr expr = u[i] - intervention.start_window;
            expr += (intervention.start_window + intervention.duration + instance.time_matrix[vehicle.interventions[i]][vehicle.interventions[j]]) * y[i][j];
            model.addConstr(expr <= u[j]);
        }
    }
    // Depot time constraints
    for (int i = 0; i < n_intervention; i++) {
        int duration = instance.nodes[vehicle.interventions[i]].duration;
        model.addConstr(instance.time_matrix[vehicle.depot][vehicle.interventions[i]] * y[n_nodes - 1][i] <= u[i]);
        model.addConstr(u[i] + duration + instance.time_matrix[vehicle.interventions[i]][vehicle.depot] * y[i][n_nodes - 1] <= END_DAY);
    }
    // Lunch break
    for (int i = 0; i < n_intervention; i++) {
        int duration = instance.nodes[vehicle.interventions[i]].duration;
        model.addConstr(u[i] + duration <= MID_DAY + (END_DAY - MID_DAY) * z[i]);
        model.addConstr(u[i] >= MID_DAY * z[i]);
    }
    // No need to check the capacities - order is capacity agnostic

    // Objective function
    GRBLinExpr obj = 0;
    for (int i = 0; i < n_nodes; i++) {
        for (int j = 0; j < n_nodes; j++) {
            obj += instance.distance_matrix[vehicle.interventions[i]][vehicle.interventions[j]] * y[i][j] * instance.cost_per_km;
        }
    }
    model.setObjective(obj, GRB_MINIMIZE);

    // Optimize the model
    model.optimize();

    // Get the results
    vector<int> id_sequence;
    

    



}