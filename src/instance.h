// Definition of the various classes used to represent an instance of the problem
#pragma once
#include <map>
#include <vector>
#include <string>
#include <set>

/*
    @struct Node
    @brief Represents an intervention or a warehouse in the problem
    @param id: unique string identifier for the node
    @param node_id: index of the node in the time and distance matrices
    @param is_intervention: type of the node
    @param duration: duration of the intervention
    @param start_window: start of the time window for the intervention
    @param end_window: end of the time window for the intervention
    @param is_ambiguous: can the intervention be done both in the morning or in the afternoon?
    @param quantities: quantities of each ressource used by the intervention
    @param required_skills: skills required to perform the intervention (left empty for warehouses)
*/
struct Node {
    // id is a unique string identifier for the node
    std::string id;
    // node_id is the index of the node in the time and distance matrice
    int node_id;
    // Type of the node
    bool is_intervention;
    // Duration of the intervention
    int duration;
    // Time window for the intervention
    int start_window;
    int end_window;
    // Can the intervention be done both in the morning or in the afternoon?
    bool is_ambiguous;
    // Quantities of each ressource used by the intervention
    // The ressources help prevent using up all the time doing interventions (to leave flexibility in the planning)
    std::map<std::string, int> quantities;
    // Skills required to perform the intervention (left empty for warehouses) (counting the number of technicians needed with each skill)
    std::map<std::string, int> required_skills;
};

/*
    @struct Technician
    @brief Represents a technician in the problem
    @param id: unique string identifier for the technician
    @param operationnal_base: base of the technician
    @param skills: set of skills the technician has
    @param capacities: capacities of the technician
*/
struct Technician {
    std::string id;
    std::string operationnal_base;
    std::set<std::string> skills;
    std::map<std::string, int> capacities;
};

/*
    @struct Vehicle
    @brief Represents a vehicle in the problem
    @param id: unique identifier for the vehicle
    @param technicians: vector containing the ids of the technicians in the vehicle
    @param skills: counts the number of technicians that have each skill in the vehicle
    @param interventions: list of indexes of interventions that the vehicle can perform
    @param reverse_interventions: reverse index of the interventions (from the nodes vector to the interventions vector)
    @param depot: index of the depot in the nodes vector
    @param capacities: capacities of the vehicle
    @param cost: cost of using the vehicle
*/
struct Vehicle {
    int id;
    // Vector containing the ids of the technicians in the vehicle
    std::vector<std::string> technicians;
    // Counts the number of technicians that have each skill in the vehicle
    std::map<std::string, int> skills; 
    // List of indexes of interventions that the vehicle can perform
    // (indexes with respect to the nodes std::vector in the instance)
    std::vector<int> interventions;
    // Reverse index of the interventions (from the nodes std::vector to the interventions vector)
    std::map<int, int> reverse_interventions;
    // Index of the depot in the nodes std::vector
    int depot;
    std::map<std::string, int> capacities;
    double cost;
};

// Generate a new vehicle from an existing one
// We only keep in the interventions those that have a mask value of 0
Vehicle vehicle_mask(const Vehicle& vehicle, const std::vector<int>& mask);


/*
    @struct Instance
    @brief Represents an instance of the problem
    @param number_interventions: number of interventions in the instance
    @param number_warehouses: number of warehouses in the instance
    @param number_vehicles: number of vehicles in the instance
    @param cost_per_km: cost per km for the vehicles
    @param technician_cost: cost of using a technician
    @param M: Big M used in the objective function
    @param nodes: std::vector of interventions and warehouses
    @param vehicles: std::vector of vehicles
    @param capacities_labels: different capacities considered
    @param time_matrix: time between the different nodes
    @param distance_matrix: distance between the different nodes
*/
struct Instance {
    int number_interventions;
    int number_warehouses;
    int number_vehicles;
    double cost_per_km;
    double technician_cost;
    // Big M used in the objective function
    double M;
    // std::vector of interventions and warehouses
    std::vector<Node> nodes;
    // std::vector of vehicles
    std::vector<Vehicle> vehicles;
    // Different capacities considered
    std::vector<std::string> capacities_labels;
    // time between the different nodes
    std::vector<std::vector<int>> time_matrix;
    // distance between the different nodes
    std::vector<std::vector<int>> distance_matrix;
};


bool is_symmetric(const std::vector<std::vector<int>>& matrix);

int symmetry_gap(const std::vector<std::vector<int>>& matrix);