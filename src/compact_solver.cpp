#include "compact_solver.h"

#include <iostream>
#include <string>
#include "gurobi_c++.h"
#include "constants.h"



void compact_solver(const Instance & instance) {
    using std::vector, std::find;
    using std::string;

    // Build the compact formulation of the problem then solve it
    int n_interventions = instance.number_interventions;
    int n_nodes = instance.nodes.size();
    int n_vehicles = instance.number_vehicles;
    int n_resources = instance.capacities_labels.size();

    // Create the model
    GRBEnv env = GRBEnv(true);
    GRBModel model = GRBModel(env);

    // Build the 3D list of variables x_ijv
    vector<vector<vector<GRBVar>>> x(n_nodes, vector<vector<GRBVar>>(n_nodes, vector<GRBVar>(n_vehicles)));
    for (int i = 0; i < n_nodes; i++) {
        for (int j = 0; j < n_nodes; j++) {
            for (int v = 0; v < n_vehicles; v++) {
                x[i][j][v] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
            }
        }
    }
    // The list of variables y_v
    vector<GRBVar> y(n_vehicles);
    for (int v = 0; v < n_vehicles; v++) {
        y[v] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
    }
    // The variables u_i (start time of intervention i)
    vector<GRBVar> u(n_interventions);
    for (int i = 0; i < n_interventions; i++) {
        u[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS);
    }
    // The variables z_i (wether intervention i is done in the afternoon)
    vector<GRBVar> z(n_interventions);
    for (int i = 0; i < n_interventions; i++) {
        z[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
    }

    // Add the constraints
    // Each intervention is visited at most once
    vector<GRBConstr> intervention_constraints;
    for (int i = 0; i < n_interventions; i++) {
        GRBLinExpr expr = 0;
        for (int j = 0; j < n_nodes; j++) {
            for (int v = 0; v < n_vehicles; v++) {
                expr += x[i][j][v];
            }
        }
        intervention_constraints.push_back(model.addConstr(expr <= 1));
    }
    // Vehicle routing contraints
    // Can't go from a node to itself
    for (int i = 0; i < n_nodes; i++) {
        for (int v = 0; v < n_vehicles; v++) {
            model.addConstr(x[i][i][v] == 0);
        }
    }
    // Flow conservation constraints
    for (int i = 0; i < n_nodes; i++) {
        for (int v = 0; v < n_vehicles; v++) {
            GRBLinExpr expr = 0;
            for (int j = 0; j < n_nodes; j++) {
                expr += x[i][j][v] - x[j][i][v];
            }
            model.addConstr(expr == 0);
        }
    }
    // A vehicle leaves the depot only if used
    for (int v = 0; v < n_vehicles; v++) {
        int depot = instance.vehicles[v].depot;
        GRBLinExpr expr = 0;
        for (int j = 0; j < n_nodes; j++) {
            expr += x[depot][j][v];
        }
        model.addConstr(expr == y[v]);
    }
    // A vehicle can't be used if not available
    for (int i = 0; i < n_interventions; i++) {
        for (int j = 0; j < n_nodes; j++) {
            for (int v = 0; v < n_vehicles; v++) {
                model.addConstr(x[i][j][v] <= y[v]);
            }
        }
    }

    // Skill constraints
    for (int i = 0; i < n_interventions; i++) {
        for (int v = 0; v < n_vehicles; v++) {
            // Check wether vehcile v can do intervention i
            const Vehicle& vehicle = instance.vehicles[v];
            bool can_do = find(vehicle.interventions.begin(), vehicle.interventions.end(), i) != vehicle.interventions.end();
            for (int j = 0; j < n_nodes; j++) {
                model.addConstr(x[i][j][v] <= can_do);
            }
        }
    }

    // Intervention scheduling constraints
    // Respecting the time windows
    for (int i = 0; i < n_interventions; i++) {
        const Node& intervention = instance.nodes[i];
        model.addConstr(u[i] >= intervention.start_window);
        model.addConstr(u[i] + intervention.duration <= intervention.end_window);
    }
    // Respecting the travel time
    for (int i = 0; i < n_interventions; i++) {
        for (int j = 0; j < n_interventions; j++) {
            const Node& node_i = instance.nodes[i];
            const Node& node_j = instance.nodes[j];
            GRBLinExpr expr = u[i] - node_i.start_window;
            double coef = node_i.start_window + node_i.duration + metric(node_i, node_j, instance.time_matrix);
            for (int v = 0; v < n_vehicles; v++) {
                expr += coef * x[i][j][v];
            }
            model.addConstr(expr <= u[j]);
        }
    }
    // Also respect the travel time to and from the depot
    for (int i = 0; i < n_interventions; i++) {
        const Node & node_i = instance.nodes[i];
        for (int v = 0; v < n_vehicles; v++) {
            int depot = instance.vehicles[v].depot;
            const Node & depot_node = instance.nodes[depot];
            int travel_time = metric(depot_node, node_i, instance.time_matrix);
            // from the depot
            model.addConstr(travel_time * x[depot][i][v] <= u[i]);
            // to the depot
            model.addConstr(u[i] + node_i.duration + travel_time* x[i][depot][v] <= END_DAY);
        }
    }
    // Lunch break
    for (int i = 0; i < n_interventions; i++) {
        if (!instance.nodes[i].is_ambiguous) continue;
        int duration = instance.nodes[i].duration;
        model.addConstr(u[i] + duration <= MID_DAY + (END_DAY - MID_DAY) * z[i]);
        model.addConstr(u[i] >= MID_DAY * z[i]);
    }
    // Capacities constraints
    for (const string & label : instance.capacities_labels) {
        for (int v = 0; v < n_vehicles; v++) {
            GRBLinExpr expr = 0;
            for (int i = 0; i < n_interventions; i++) {
                for (int j = 0; j < n_nodes; j++) {
                    expr += instance.nodes[i].quantities.at(label) * x[i][j][v];
                }
            }
            model.addConstr(expr <= instance.vehicles[v].capacities.at(label));
        }


}   