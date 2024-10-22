#include "compact_solver.h"

#include <iostream>
#include <string>
#include "gurobi_c++.h"
#include "instance/constants.h"



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
        y[v] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS);
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
        for (int d = n_interventions; d < n_nodes; d++) {
            if (d == real_depot) continue;
            GRBLinExpr expr_in = 0;
            GRBLinExpr expr_out = 0;
            string name = "vehicle_" + std::to_string(v) + "not_going_to_depot_" + std::to_string(d);
            for (int i = 0; i < n_nodes; i++) {
                expr_in += x[i][d][v];
                expr_out += x[d][i][v];
            }
            model.addConstr(expr_in == 0, name + "_in");
            model.addConstr(expr_out == 0, name + "_out");
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
        string name = "intervention_" + std::to_string(i) + "_visited_once";
        model.addConstr(expr <= 1, name);
    }
    // Vehicle routing contraints
    // Can't go from a node to itself
    for (int i = 0; i < n_nodes; i++) {
        for (int v = 0; v < n_vehicles; v++) {
            string name = "vehicle_" + std::to_string(v) + "not_i_to_i-i=" + std::to_string(i);
            model.addConstr(x[i][i][v] == 0, name);
        }
    }
    // Flow conservation constraints
    for (int i = 0; i < n_nodes; i++) {
        for (int v = 0; v < n_vehicles; v++) {
            GRBLinExpr expr = 0;
            for (int j = 0; j < n_nodes; j++) {
                expr += x[i][j][v] - x[j][i][v];
            }
            string name = "vehicle_" + std::to_string(v) + "_flow_conservation_i=" + std::to_string(i);
            model.addConstr(expr == 0, name);
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
        string name = "vehicle_" + std::to_string(v) + "_uses_depot";
        model.addConstr(expr_out == y[v], name + "_out");
        model.addConstr(expr_in == y[v], name + "_in");
    }
    // A vehicle can't be used if not available
    for (int i = 0; i < n_nodes; i++) {
        for (int j = 0; j < n_nodes; j++) {
            for (int v = 0; v < n_vehicles; v++) {
                string name = "vehicle_" + std::to_string(v) + "_available_i=" + std::to_string(i) + "_j=" + std::to_string(j);
                model.addConstr(x[i][j][v] <= y[v], name);
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
                string name = "vehicle_" + std::to_string(v) + "_can_do_intervention_" + std::to_string(i);
                model.addConstr(x[i][j][v] <= can_do, name);
            }
        }
    }

    // Intervention scheduling constraints
    // Respecting the time windows
    for (int i = 0; i < n_interventions; i++) {
        const Node& intervention = instance.nodes[i];
        string name = "intervention_" + std::to_string(i) + "_time_window";
        model.addConstr(u[i] >= intervention.start_window, name + "_start");
        // The time window might be ill defined and too narrow
        if (intervention.start_window + intervention.duration <= intervention.end_window) {
            model.addConstr(u[i] + intervention.duration <= intervention.end_window, name + "_end");
        } else {
            // If the time window is too narrow, we can't respect it, no vehicle can do the intervention
            for (int j = 0; j < n_nodes; j++) {
                for (int v = 0; v < n_vehicles; v++) {
                    model.addConstr(x[i][j][v] == 0);
                }
            }
        }
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
            string name = "intervention_" + std::to_string(i) + "_to_" + std::to_string(j) + "_travel_time";
            model.addConstr(expr <= u[j], name);
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
            string name = "travel_time_intervention-depot_i=" + std::to_string(i) + "_v=" + std::to_string(v);
            // from the depot
            model.addConstr(travel_time_out * x[depot][i][v] <= u[i], name + "_out");
            // to the depot
            model.addConstr(u[i] + node_i.duration + travel_time_in* x[i][depot][v] <= END_DAY, name + "_in");
        }
    }
    // Lunch break
    for (int i = 0; i < n_interventions; i++) {
        if (!instance.nodes[i].is_ambiguous) continue;
        int duration = instance.nodes[i].duration;
        string name = "lunch_break_intervention_" + std::to_string(i);
        model.addConstr(u[i] + duration <= MID_DAY + (END_DAY - MID_DAY) * z[i], name + "morning");
        model.addConstr(u[i] >= MID_DAY * z[i], name + "afternoon");
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
            string name = "vehicle_" + std::to_string(v) + "_capacity_" + label;
            model.addConstr(expr <= instance.vehicles[v].capacities.at(label), name);
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

    // If the model is infeasible, dump the IIS and return an empty solution
    if (model.get(GRB_IntAttr_Status) == GRB_INFEASIBLE || model.get(GRB_IntAttr_Status) == GRB_INF_OR_UNBD) {
        model.computeIIS();
        std::cout << "The following constraint(s) cannot be satisfied:" << std::endl;
        auto c = model.getConstrs();
        for (int i = 0; i < model.get(GRB_IntAttr_NumConstrs); ++i)
        {
            if (c[i].get(GRB_IntAttr_IISConstr) == 1)
            {
                std::cout << c[i].get(GRB_StringAttr_ConstrName) << std::endl;
            }
        }

        std::string filename = "results/" + instance.name + "_infeasible.ilp";
        model.write(filename);
        return CompactSolution<double>();
    }

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
}   