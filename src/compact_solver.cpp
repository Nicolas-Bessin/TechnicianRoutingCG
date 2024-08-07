#include "compact_solver.h"

#include <iostream>
#include <string>
#include "gurobi_c++.h"
#include "constants.h"



CompactSolution<int> compact_solver(const Instance & instance, int time_limit, std::vector<Route> routes, int mode, bool verbose) {
    using std::vector, std::find;
    using std::pair;
    using std::string;
        
    // Build the compact formulation of the problem then solve it
    int n_interventions = instance.number_interventions;
    int n_nodes = instance.nodes.size();
    int n_vehicles = instance.number_vehicles;
    int n_resources = instance.capacities_labels.size();

    try {
        // Create the model
        GRBEnv env = GRBEnv(true);
        env.start();
        // Set the time limit
        env.set(GRB_DoubleParam_TimeLimit, time_limit);
        // Set the verbosity
        if (!verbose) {
            env.set(GRB_IntParam_OutputFlag, 0);
        }
        
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
            u[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
        }
        // The variables z_i (wether intervention i is done in the afternoon)
        vector<GRBVar> z(n_interventions);
        for (int i = 0; i < n_interventions; i++) {
            z[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
        }

        // Add the constraints
        // Custom constraint : you cannot go into a depot that is not your own, and you cannot go out of a depot that is not your own
        for (int v = 0; v < n_vehicles; v++) {
            int real_depot = instance.vehicles[v].depot;
            for (int i = 0; i < n_nodes; i++) {
                for (int j = n_interventions; j < n_nodes; j++) {
                    if (j == real_depot) continue;
                    model.addConstr(x[i][j][v] == 0);
                    model.addConstr(x[j][i][v] == 0);
                }
            }
        }
        //Each intervention is visited at most once
        for (int i = 0; i < n_interventions; i++) {
            GRBLinExpr expr = 0;
            for (int j = 0; j < n_nodes; j++) {
                for (int v = 0; v < n_vehicles; v++) {
                    expr += x[i][j][v];
                }
            }
            model.addConstr(expr <= 1);
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
            GRBLinExpr expr_out = 0;
            GRBLinExpr expr_in = 0;
            for (int j = 0; j < n_nodes; j++) {
                expr_out += x[depot][j][v];
                expr_in += x[j][depot][v];
            }
            model.addConstr(expr_out == y[v]);
            model.addConstr(expr_in == y[v]);
        }
        // A vehicle can't be used if not available
        for (int i = 0; i < n_nodes; i++) {
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
                int can_do = find(vehicle.interventions.begin(), vehicle.interventions.end(), i) != vehicle.interventions.end();
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
                GRBLinExpr expr = u[i] - node_i.end_window;
                double coef = node_i.end_window + node_i.duration + instance.time_matrix[i][j];
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
                int travel_time_out = instance.time_matrix[depot][i];
                int travel_time_in = instance.time_matrix[i][depot];
                // from the depot
                model.addConstr(travel_time_out * x[depot][i][v] <= u[i]);
                // to the depot
                model.addConstr(u[i] + node_i.duration + travel_time_in* x[i][depot][v] <= END_DAY);
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

        // Mode specific constraints
        if (mode == WARM_START) {
            // Warm start : we use the routes to set the variables
            for (const Route & route : routes) {
                int v = route.vehicle_id;
                for (int i = 0; i < route.id_sequence.size() - 1; i++) {
                    int node_i = route.id_sequence[i];
                    int node_j = route.id_sequence[i+1];
                    x[node_i][node_j][v].set(GRB_DoubleAttr_Start, 1.0);
                }
            }
        }
        if (mode == IMPOSE_ROUTING) {
            // Dummy integer solution : we use all the routes
            IntegerSolution integer_solution = IntegerSolution(vector<int>(routes.size(), 1), -1);
            vector<pair<int, int>> imposed_routings = imposed_routings_from_routes(routes, integer_solution);
            // Finally, we can set the imposed routings
            // Each pair is a pair (vehicle_id, intervention_id)
            // Where the vehicle has to go through the intervention
            for (const auto& [v, i] : imposed_routings) {
                GRBLinExpr expr = 0;
                for (int j = 0; j < n_nodes; j++) {
                    expr += x[i][j][v];
                }
                model.addConstr(expr == 1);
            }
        }
            

        // Set the objective function
        GRBLinExpr obj = 0;
        for (int i = 0; i < n_nodes; i++) {
            const Node & node_i = instance.nodes[i];
            for (int j = 0; j < n_nodes; j++) {
                const Node & node_j = instance.nodes[j];
                int distance = instance.distance_matrix[i][j];
                for (int v = 0; v < n_vehicles; v++) {
                    obj += (instance.M * node_i.duration - distance * instance.cost_per_km) * x[i][j][v];
                }
            }
        }
        // Substract the cost of the vehicles
        for (int v = 0; v < n_vehicles; v++) {
            obj -= instance.vehicles[v].cost * y[v];
        }
        model.setObjective(obj, GRB_MAXIMIZE);

        // Optimize the model
        model.optimize();

        // Build the solution
        CompactSolution<int> solution = CompactSolution<int>(n_nodes, n_interventions, n_vehicles);
        solution.objective_value = model.get(GRB_DoubleAttr_ObjVal);

        // Get the values of the variables x_ijv
        for (int i = 0; i < n_nodes; i++) {
            for (int j = 0; j < n_nodes; j++) {
                for (int v = 0; v < n_vehicles; v++) {
                    solution.x[i][j][v] = x[i][j][v].get(GRB_DoubleAttr_X);
                }
            }
        }
        // Get the values of the variables y_v
        for (int v = 0; v < n_vehicles; v++) {
            solution.y[v] = y[v].get(GRB_DoubleAttr_X);
        }
        // Get the values of the variables u_i
        for (int i = 0; i < n_interventions; i++) {
            solution.u[i] = u[i].get(GRB_DoubleAttr_X);
        }

        return solution;


    } catch (GRBException & e) {
        std::cerr << "Error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;
    } catch (...) {
        std::cerr << "Exception during optimization" << std::endl;
    }

    return CompactSolution<int>();
}   



CompactSolution<double> relaxed_compact_solver(const Instance & instance, int time_limit, bool verbose) {
    using std::vector, std::find;
    using std::pair;
    using std::string;
        
    // Build the compact formulation of the problem then solve it
    int n_interventions = instance.number_interventions;
    int n_nodes = instance.nodes.size();
    int n_vehicles = instance.number_vehicles;
    int n_resources = instance.capacities_labels.size();

    try {
        // Create the model
        GRBEnv env = GRBEnv(true);
        env.start();
        // Set the time limit
        env.set(GRB_DoubleParam_TimeLimit, time_limit);
        // Set the verbosity
        if (!verbose) {
            env.set(GRB_IntParam_OutputFlag, 0);
        }
        
        GRBModel model = GRBModel(env);


        // Build the 3D list of variables x_ijv
        vector<vector<vector<GRBVar>>> x(n_nodes, vector<vector<GRBVar>>(n_nodes, vector<GRBVar>(n_vehicles)));
        for (int i = 0; i < n_nodes; i++) {
            for (int j = 0; j < n_nodes; j++) {
                for (int v = 0; v < n_vehicles; v++) {
                    x[i][j][v] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS);
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
            u[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_INTEGER);
        }
        // The variables z_i (wether intervention i is done in the afternoon)
        vector<GRBVar> z(n_interventions);
        for (int i = 0; i < n_interventions; i++) {
            z[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
        }

        // Add the constraints
        // Custom constraint : you cannot go into a depot that is not your own, and you cannot go out of a depot that is not your own
        for (int v = 0; v < n_vehicles; v++) {
            int real_depot = instance.vehicles[v].depot;
            for (int i = 0; i < n_nodes; i++) {
                for (int j = n_interventions; j < n_nodes; j++) {
                    if (j == real_depot) continue;
                    model.addConstr(x[i][j][v] == 0);
                    model.addConstr(x[j][i][v] == 0);
                }
            }
        }
        //Each intervention is visited at most once
        for (int i = 0; i < n_interventions; i++) {
            GRBLinExpr expr = 0;
            for (int j = 0; j < n_nodes; j++) {
                for (int v = 0; v < n_vehicles; v++) {
                    expr += x[i][j][v];
                }
            }
            model.addConstr(expr <= 1);
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
            GRBLinExpr expr_out = 0;
            GRBLinExpr expr_in = 0;
            for (int j = 0; j < n_nodes; j++) {
                expr_out += x[depot][j][v];
                expr_in += x[j][depot][v];
            }
            model.addConstr(expr_out == y[v]);
            model.addConstr(expr_in == y[v]);
        }
        // A vehicle can't be used if not available
        for (int i = 0; i < n_nodes; i++) {
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
                int can_do = find(vehicle.interventions.begin(), vehicle.interventions.end(), i) != vehicle.interventions.end();
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
                GRBLinExpr expr = u[i] - node_i.end_window;
                double coef = node_i.end_window + node_i.duration + instance.time_matrix[i][j];
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
                int travel_time_out = instance.time_matrix[depot][i];
                int travel_time_in = instance.time_matrix[i][depot];
                // from the depot
                model.addConstr(travel_time_out * x[depot][i][v] <= u[i]);
                // to the depot
                model.addConstr(u[i] + node_i.duration + travel_time_in* x[i][depot][v] <= END_DAY);
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

        // Set the objective function
        GRBLinExpr obj = 0;
        for (int i = 0; i < n_nodes; i++) {
            const Node & node_i = instance.nodes[i];
            for (int j = 0; j < n_nodes; j++) {
                const Node & node_j = instance.nodes[j];
                int distance = instance.distance_matrix[i][j];
                for (int v = 0; v < n_vehicles; v++) {
                    obj += (instance.M * node_i.duration - distance * instance.cost_per_km) * x[i][j][v];
                }
            }
        }
        // Substract the cost of the vehicles
        for (int v = 0; v < n_vehicles; v++) {
            obj -= instance.vehicles[v].cost * y[v];
        }
        model.setObjective(obj, GRB_MAXIMIZE);

        // Optimize the model
        model.optimize();

        // Build the solution
        CompactSolution<double> solution = CompactSolution<double>(n_nodes, n_interventions, n_vehicles);
        solution.objective_value = model.get(GRB_DoubleAttr_ObjVal);

        // Get the values of the variables x_ijv
        for (int i = 0; i < n_nodes; i++) {
            for (int j = 0; j < n_nodes; j++) {
                for (int v = 0; v < n_vehicles; v++) {
                    solution.x[i][j][v] = x[i][j][v].get(GRB_DoubleAttr_X);
                }
            }
        }
        // Get the values of the variables y_v
        for (int v = 0; v < n_vehicles; v++) {
            solution.y[v] = y[v].get(GRB_DoubleAttr_X);
        }
        // Get the values of the variables u_i
        for (int i = 0; i < n_interventions; i++) {
            solution.u[i] = u[i].get(GRB_DoubleAttr_X);
        }

        return solution;


    } catch (GRBException & e) {
        std::cerr << "Error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;
    } catch (...) {
        std::cerr << "Exception during optimization" << std::endl;
    }

    return CompactSolution<double>();
}   