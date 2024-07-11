#include "parser.h"
#include "constants.h"
#include "../nlohmann/json.hpp"
#include <fstream>
#include <iostream>

using json = nlohmann::json;

// Convert a start time into a relative time 
// 0 is the beginning of the work day
// Lunch break not included in the work time measurement
int convert_start_time(int start_time){
    if (start_time < START_MORNING){
        return START_MORNING;
    } else if (start_time < END_MORNING){
        return start_time - START_MORNING;
    } else if (start_time < END_AFTERNOON){
        return start_time - START_MORNING - LUNCH_BREAK;
    } else {
        return END_DAY;
    }
}

// Convert an end time into a relative time
// 0 is the beginning of the work day
// Lunch break not included in the work time measurement
int convert_end_time(int end_time){
    if (end_time < START_MORNING){
        return 0;
    } else if (end_time < END_MORNING){
        return end_time - START_MORNING;
    } else if (end_time < END_AFTERNOON){
        return end_time - START_MORNING - LUNCH_BREAK;
    } else {
        return END_DAY;
    }
}

// Parse a JSON object into an Intervention object
Intervention parse_intervention(json data){
    string id = data.at("id");
    int node_id = data.at("node_id");
    int duration = data.at("duration");
    int start_window = convert_start_time(data.at("start_window"));
    int end_window = convert_end_time(data.at("end_window"));
    bool is_long = duration > 120;
    set<string> skills = set<string>();
    // In the JSON object, skills are stored as lists of lists of strings
    for (auto skill : data.at("skills")){
        for (auto s : skill){
            skills.insert(s);
        }
    }
    map<string, int> quantities = data.at("quantities");
    return Intervention(id, node_id, duration, start_window, end_window, is_long, skills, quantities);
}


// Parse a JSON object into a Warehouse object
Warehouse parse_warehouse(json data){
    int node_id = data.at("node_id");
    string ope_base = data.at("ope_base");
    return Warehouse(ope_base, node_id);
}


// Parse a technician JSON object into a Technician object
Technician parse_technician(json data){
    string id = data.at("id");
    set<string> skills = set<string>();
    for (auto skill : data.at("skills")){
        skills.insert(skill);
    }
    map<string, int> capacities = data.at("capacities");
    string operationnal_base = data.at("ope_base");
    return Technician(id, skills, capacities, operationnal_base);
}


// Parse a JSON file to return a Instance object
Instance parse_file(string filename){
    cout << "Parsing file " << filename << endl;

    // Read the JSON file
    ifstream f(filename);
    json data = json::parse(f);

    // Get the main constants
    json constants = data.at("const_manager");
    double cost_per_km = constants.at("km_cost");
    double tech_cost = constants.at("tech_cost");
    int number_ressources = constants.at("capacities_size");
    vector<string> ressources = constants.at("capacities_labels");

    // Print the main constants
    cout << "Cost per km: " << cost_per_km << endl;
    cout << "Technician cost: " << tech_cost << endl;

    // Get the matrices of distance and time
    json loc_manager = data.at("loc_manager");
    vector<vector<double>> distance_matrix = loc_manager.at("matrix").at("distance");
    vector<vector<double>> time_matrix = loc_manager.at("matrix").at("time");

    // For the interventions and warehouse, we wil put them into a single vector
    // Also build a map from intervention / warehouse id to index in the vector
    vector<Node> nodes = vector<Node>();
    map<string, int> node_id_to_index = map<string, int>();

    // Get the interventions
    vector<json> interventions_data = data.at("step_manager").at("interventions");
    for (int i = 0; i < interventions_data.size(); i++){
        Intervention intervention = parse_intervention(interventions_data[i]);
        nodes.push_back(intervention);
        node_id_to_index[intervention.id] = i;
    }

    // Print the number of interventions
    int nb_interventions = nodes.size();
    cout << "Number of interventions: " << nb_interventions << endl;

    // Get the warehouses and add them to the nodes
    vector<json> warehouses_data = data.at("step_manager").at("warehouses");
    for (int i = 0; i < warehouses_data.size(); i++){
        Warehouse warehouse = parse_warehouse(warehouses_data[i]);
        nodes.push_back(warehouse);
        node_id_to_index[warehouse.id] = i + nb_interventions;
    }

    // Print the number of warehouses and number of nodes
    int nb_warehouses = warehouses_data.size();
    cout << "Number of warehouses: " << nb_warehouses << endl;
    cout << "Number of nodes: " << nodes.size() << endl;


    // Before building the teams, we put all the technicians into a dictionary for easy access
    vector<json> technicians_data = data.at("tech_manager").at("technicians");
    map<string, Technician> technicians = map<string, Technician>();
    for (int i = 0; i < technicians_data.size(); i++){
        Technician tech = parse_technician(technicians_data[i]);
        technicians[tech.id] = tech;
    }

    // Print the number of technicians
    cout << "Number of technicians: " << technicians.size() << endl;

    // Teams are groups of technicians
    // Initially, we have fixed teams
    vector<vector<string>> tech_id_per_team = data.at("tech_manager").at("teams").at("fixed_teams");
    // Also keep track of the technicians already in a team
    set<string> tech_in_team = set<string>();
    for (int i = 0; i < tech_id_per_team.size(); i++){
        for (int j = 0; j < tech_id_per_team[i].size(); j++){
            tech_in_team.insert(tech_id_per_team[i][j]);
        }
    }
    // We then add every technician not into a fixed team as a team of one
    for (auto tech : technicians){
        if (tech_in_team.find(tech.first) == tech_in_team.end()){
            tech_id_per_team.push_back(vector<string>{tech.first});
        }
    }
    
    // Print the number of teams
    cout << "Number of teams: " << tech_id_per_team.size() << endl;

    // We now construct one vehicle per team
    vector<Vehicle> vehicles = vector<Vehicle>();
    for (int i = 0; i < tech_id_per_team.size(); i++){
        // Build the available skills for the vehicle
        set<string> skills = set<string>();
        for (int j = 0; j < tech_id_per_team[i].size(); j++){
            for (auto skill : technicians[tech_id_per_team[i][j]].skills){
                skills.insert(skill);
            }
        }
        // We will build the list of interventions for the vehicle later
        vector<Node*> interventions = vector<Node*>();
        // Check that every technician in the team has the same operationnal base
        string operationnal_base = technicians[tech_id_per_team[i][0]].operationnal_base;
        for (int j = 1; j < tech_id_per_team[i].size(); j++){
            if (technicians[tech_id_per_team[i][j]].operationnal_base != operationnal_base){
                cout << "Warning : technicians in the same team must have the same operationnal base - team " << i << endl;
                cout << "Technician " << tech_id_per_team[i][j] << " has operationnal base " << technicians[tech_id_per_team[i][j]].operationnal_base << endl;
                cout << "Technician " << tech_id_per_team[i][0] << " has operationnal base " << technicians[tech_id_per_team[i][0]].operationnal_base << endl;
            }
        }
        // The depot is the index of the operationnal base in the nodes vector
        int depot = node_id_to_index[operationnal_base];
        // Build the capacities of the vehicles as the min of the capacities each technician for each ressource
        map<string, int> capacities = map<string, int>();
        for (int j = 0; j < number_ressources; j++){
            string ressource = ressources[j];
            int min_capacity = technicians[tech_id_per_team[i][0]].capacities[ressource];
            for (int k = 1; k < tech_id_per_team[i].size(); k++){
                min_capacity = min(min_capacity, technicians[tech_id_per_team[i][k]].capacities[ressource]);
            }
            capacities[ressource] = min_capacity;
        }
        // Vehicle cost is the sum of the cost of each technician in the team
        double vehicle_cost = tech_cost * tech_id_per_team[i].size();
        vehicles.push_back(Vehicle(i, skills, interventions, depot, capacities, START_MORNING, END_AFTERNOON, END_DAY, vehicle_cost));
    }

    // Print the number of vehicles
    int nb_vehicles = vehicles.size();
    cout << "Number of vehicles: " << nb_vehicles << endl;

    // We now build a cross reference matrix between interventions and vehicles : has_skill[i][j] is true if vehicle j has the skills to do intervention i
    vector<vector<bool>> has_skill = vector<vector<bool>>(nb_interventions, vector<bool>(nb_vehicles, false));
    for (int i = 0; i < nb_interventions; i++){
        for (int j = 0; j < nb_vehicles; j++){
            bool has_all_skills = true;
            for (auto skill : nodes[i].required_skills){
                bool has_skill = vehicles[j].skills.contains(skill);
                has_all_skills = has_all_skills && has_skill;
            }
            has_skill[i][j] = has_all_skills;
        }
    }

    // We can now count the number of vehicles that can do each intervention
    for (int i = 0; i < nb_interventions; i++){
        int nb_vehicles_for_intervention = 0;
        for (int j = 0; j < nb_vehicles; j++){
            if (has_skill[i][j]){
                nb_vehicles_for_intervention++;
            }
        }
        cout << "Intervention " << i << " can be done by " << nb_vehicles_for_intervention << " vehicles" << endl;
        // Update this count in the corresponding intervention
        nodes[i].nb_vehicles = nb_vehicles_for_intervention;
    }

    // And we can add references to the interventions that each vehicle can do
    for (int v = 0; v < nb_vehicles; v++){
        for (int i = 0; i < nb_interventions; i++){
            if (has_skill[i][v]){
                vehicles[v].interventions.push_back(&nodes[i]);
            }
        }
    }

    // We can now build the instance object
    return Instance(
        nb_interventions,
        nb_warehouses,
        nb_vehicles,
        cost_per_km,
        tech_cost,
        node_id_to_index,
        nodes,
        vehicles,
        ressources,
        time_matrix,
        distance_matrix
    );    

}
    
