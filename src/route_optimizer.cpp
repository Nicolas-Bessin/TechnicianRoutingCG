#include "route_optimizer.h"

#include "analysis.h"
#include "solution_converter.h"

#include <vector>
#include "gurobi_c++.h"

Route optimize_route(const Route& route, const Instance& instance) {
    using namespace std;

    // Get the vehicle that will perform the route
    // Only keep the interventions that are in the route
    const Vehicle vehicle = vehicle_mask(instance.vehicles[route.vehicle_id], route.is_in_route, KEEP_COVERED);

    int n_nodes = vehicle.interventions.size() + 1;
    int n_intervention = vehicle.interventions.size();

    try {
        // Create the model
        GRBEnv env = GRBEnv(true);
        env.set(GRB_IntParam_OutputFlag, 0);
        env.start();
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
            u[i] = model.addVar(0.0, END_DAY, 0.0, GRB_INTEGER);
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
                GRBLinExpr expr = u[i] - intervention.end_window;
                expr += (intervention.end_window + intervention.duration + instance.time_matrix[vehicle.interventions[i]][vehicle.interventions[j]]) * y[i][j];
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
        // No need to check the capacities - capacity is order agnostic, and the initial route is feasible

        // Objective function
        GRBLinExpr obj = 0;
        for (int i = 0; i < n_nodes; i++) {
            for (int j = 0; j < n_nodes; j++) {
                int true_i = i == n_nodes - 1 ? vehicle.depot : vehicle.interventions[i];
                int true_j = j == n_nodes - 1 ? vehicle.depot : vehicle.interventions[j];
                obj += instance.distance_matrix[true_i][true_j] * y[i][j] * instance.cost_per_km;
            }
        }
        model.setObjective(obj, GRB_MINIMIZE);

        // Optimize the model
        model.optimize();

        // Get the results - as a CompactSolution
        CompactSolution<int> compact_solution(instance.nodes.size(), instance.number_interventions, instance.vehicles.size());
        compact_solution.objective_value = model.get(GRB_DoubleAttr_ObjVal);
        // Set the x variables in the compact solution
        for (int i = 0; i < n_nodes; i++) {
            for (int j = 0; j < n_nodes; j++) {
                int true_i = i == n_nodes - 1 ? vehicle.depot : vehicle.interventions[i];
                int true_j = j == n_nodes - 1 ? vehicle.depot : vehicle.interventions[j];
                compact_solution.x[true_i][true_j][route.vehicle_id] = y[i][j].get(GRB_DoubleAttr_X);
                //cout << "x[" << true_i << "][" << true_j << "][" << route.vehicle_id << "] = " << compact_solution.x[true_i][true_j][route.vehicle_id] << endl;
            }
        }
        // Set the y variables in the compact solution
        compact_solution.y[route.vehicle_id] = 1;
        // Set the u variables in the compact solution
        for (int i = 0; i < n_intervention; i++) {
            compact_solution.u[vehicle.interventions[i]] = u[i].get(GRB_DoubleAttr_X);
        }

        // Convert the compact solution to a route
        // cout << "Current vehicle v" << vehicle.id << " has depot " << vehicle.depot << endl;
        vector<Route> routes = compact_solution_to_routes(instance, compact_solution);
        return routes.at(0);
    } catch (GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
        return route;
    } catch (...) {
        cout << "Exception during optimization" << endl;
        return route;
    }

    

    



}