#include "tabu.h"

#include "instance/constants.h"
#include "gurobi_c++.h"


std::vector<int> local_matrix_to_seq(const std::vector<std::vector<int>> & matrix, const Vehicle & vehicle){
    std::vector<int> sequence;
    int reduced_size = vehicle.interventions.size() + 1;
    int reduced_depot = vehicle.interventions.size();

    int current_reduced = reduced_depot;
    sequence.push_back(vehicle.depot);
    bool depot_reached = false;
    while (!depot_reached){
        int next = -1;
        for (int i = 0; i < reduced_size; i++){
            if (matrix[current_reduced][i] == 1){
                next = i;
                break;
            }
        }
        if (next == reduced_depot){
            depot_reached = true;
            sequence.push_back(vehicle.depot);
        } else {
            sequence.push_back(vehicle.interventions[next]);
            current_reduced = next;
        }
    }

    return sequence;
}

std::vector<std::vector<int>> local_matrix_to_global(
    const std::vector<std::vector<int>> & matrix, 
    const Vehicle & vehicle, 
    const Instance & instance){
    std::vector<std::vector<int>> global_matrix(instance.nodes.size(), std::vector<int>(instance.nodes.size(), 0));

    int reduced_size = vehicle.interventions.size() + 1;
    int reduced_depot = vehicle.interventions.size();

    for (int i = 0; i < reduced_size; i++){
        for (int j = 0; j < reduced_size; j++){
            if (matrix[i][j] == 1){
                int true_i = i == reduced_depot ? vehicle.depot : vehicle.interventions[i];
                int true_j = j == reduced_depot ? vehicle.depot : vehicle.interventions[j];
                global_matrix[true_i][true_j] = 1;
            }
        }
    }

    return global_matrix;
}

std::vector<int> sequence_to_mask(const std::vector<int> & sequence, const Vehicle & vehicle, const Instance & instance){
    std::vector<int> mask(instance.nodes.size(), 0);
    for (int i : sequence){
        mask[i] = 1;
    }
    return mask;
}


std::vector<Route> tabu_search(
    const Route & initial_route,
    int max_iterations,
    int max_modifications,
    const DualSolution & solution,
    const Vehicle & vehicle,
    const Instance & instance
){
    using std::vector;

    int n_interventions = vehicle.interventions.size();
    int n_nodes = vehicle.interventions.size() + 1;
    int reduced_depot = vehicle.interventions.size(); // The depot is the last node (in the reduced graph)

    // Initialize the tabu list
    vector<Route> tabu_list = {initial_route};
    Route previous_route = initial_route;

    // Create the model
    GRBEnv env = GRBEnv(true);
    env.set(GRB_IntParam_OutputFlag, 0);
    env.start();
    GRBModel model = GRBModel(env);
    // Create the variables
    vector<vector<GRBVar>> x(n_nodes, vector<GRBVar>(n_nodes));
    for (int i = 0; i < n_nodes; i++){
        for (int j = 0; j < n_nodes; j++){
            x[i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
        }
    }
    vector<GRBVar> u(n_interventions);
    for (int i = 0; i < n_nodes; i++){
        u[i] = model.addVar(0.0, n_nodes, 0.0, GRB_INTEGER);
    }
    vector<GRBVar> z(n_interventions);
    for (int i = 0; i < n_nodes; i++){
        z[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
    }
    // Constraints
    // Flow conservation
    for (int i = 0; i < n_interventions; i++){
        model.addConstr(x[i][i] <= 0);
        GRBLinExpr flow_in = 0;
        GRBLinExpr flow_out = 0;
        for (int j = 0; j < n_nodes; j++){
            flow_in += x[j][i];
            flow_out += x[i][j];
        }
        model.addConstr(flow_in - flow_out == 0);
    }
    // Depot constraint
    GRBLinExpr depot_in = 0;
    GRBLinExpr depot_out = 0;
    for (int i = 0; i < n_interventions; i++){
        depot_in += x[i][reduced_depot];
        depot_out += x[reduced_depot][i];
    }
    model.addConstr(depot_in == 1);
    model.addConstr(depot_out == 1);
    // Subtour elimination is handled by the next constraints
    // Time window constraints
    for (int i = 0; i < n_interventions; i++){
        int start_window = instance.nodes[vehicle.interventions[i]].start_window;
        int duration = instance.nodes[vehicle.interventions[i]].duration;
        int end_window = instance.nodes[vehicle.interventions[i]].end_window;
        for (int j = 0; j < n_interventions; j++){
            int time = instance.time_matrix[vehicle.interventions[i]][vehicle.interventions[j]];
            model.addConstr(u[i] - end_window + (end_window + duration + time) * x[i][j] <= u[j]);
        }
        model.addConstr(start_window <= u[i]);
        model.addConstr(u[i] + duration <= end_window);
        model.addConstr(u[i] >= MID_DAY * z[i]);
        model.addConstr(u[i] + duration <= MID_DAY + (END_DAY - MID_DAY) * z[i]);
    }
    // Depot time window constraints
    for (int i = 0; i < n_interventions; i++){
        int time_out = instance.time_matrix[vehicle.interventions[i]][vehicle.depot];
        model.addConstr(time_out * x[reduced_depot][i] <= MID_DAY);
        int duration = instance.nodes[vehicle.interventions[i]].duration;
        int time_in = instance.time_matrix[vehicle.depot][vehicle.interventions[i]];
        model.addConstr(u[i] + duration + time_in * x[i][reduced_depot] <= END_DAY);
    }
    // Objective function
    GRBLinExpr obj = -vehicle.cost;
    for (int i = 0; i < n_nodes; i++){
        for (int j = 0; j < n_nodes; j++){
            int true_i = i == reduced_depot ? vehicle.depot : vehicle.interventions[i];
            int true_j = j == reduced_depot ? vehicle.depot : vehicle.interventions[j];
            obj -= instance.time_matrix[true_i][true_j] * x[i][j] * instance.cost_per_km;
            obj += instance.nodes[true_i].duration * x[i][j] * instance.M;
            obj -= solution.alphas[true_i] * x[i][j];
        }
    }

    model.setObjective(obj, GRB_MAXIMIZE);

    // Warm start with the initial route
    for (int i = 0; i < previous_route.id_sequence.size() - 1; i++){
        int true_i = previous_route.id_sequence[i];
        int true_j = previous_route.id_sequence[i + 1];
        int reduced_i = true_i == vehicle.depot ? reduced_depot : vehicle.reverse_interventions.at(true_i);
        int reduced_j = true_j == vehicle.depot ? reduced_depot : vehicle.reverse_interventions.at(true_j);
        x[reduced_i][reduced_j].set(GRB_DoubleAttr_Start, 1.0);
    }

    // Tabu search loop
    int iteration = 0;
    while( previous_route.reduced_cost > 1e-6 && iteration < max_iterations) {
        // Add the "no good cuts" constraint relative to the previous route
        GRBLinExpr no_good_cuts = 0;
        for (int i = 0; i < n_nodes; i++){
            for (int j = 0; j < n_nodes; j++){
                int true_i = i == reduced_depot ? vehicle.depot : vehicle.interventions[i];
                int true_j = j == reduced_depot ? vehicle.depot : vehicle.interventions[j];
                if (previous_route.route_edges[true_i][true_j] == 0){
                    no_good_cuts += x[i][j];
                } else {
                    no_good_cuts += 1 - x[i][j];
                }
            }
        }
        model.addConstr(no_good_cuts <= n_nodes * n_nodes - 1);
        // Add the "at most K modifications" constraint
        auto modifications = model.addConstr(no_good_cuts <= max_modifications);
        // Optimize
        model.optimize();
        // Extract the solution
        double obj_val = model.get(GRB_DoubleAttr_ObjVal);
        // Extract the local edge matrix
        vector<vector<int>> local_matrix(n_nodes, vector<int>(n_nodes, 0));
        for (int i = 0; i < n_nodes; i++){
            for (int j = 0; j < n_nodes; j++){
                local_matrix[i][j] = x[i][j].get(GRB_DoubleAttr_X);
            }
        }
        // Convert the local matrix to a sequence
        vector<int> sequence = local_matrix_to_seq(local_matrix, vehicle);
        // Convert the local matrix to a global matrix
        vector<vector<int>> global_matrix = local_matrix_to_global(local_matrix, vehicle, instance);
        // Also get the covered interventions
        vector<int> is_in_route = sequence_to_mask(sequence, vehicle, instance);
        // Compute the reduced cost
        double reduced_cost = -obj_val - vehicle.cost;
        Route new_route = Route{
            vehicle.id,
            -1,
            reduced_cost,
            -1,
            sequence,
            is_in_route,
            global_matrix
        };
        // Update the route cost and duration
        new_route.total_cost = count_route_kilometres(new_route, instance) * instance.cost_per_km + vehicle.cost;
        new_route.total_duration = count_route_duration(new_route, instance);

        // Remove the modifications constraint
        model.remove(modifications);

        // Update the tabu list & previous route
        if (new_route.reduced_cost > 1e-6){
            tabu_list.push_back(new_route);
        }
        previous_route = new_route;

    }

    return tabu_list;

}