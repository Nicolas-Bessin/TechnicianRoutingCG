#include "preprocessing.h"
#include <iostream>

using std::cout, std::endl;

void preprocess_interventions(Instance &instance, bool verbose){
    // Number of already non-ambiguous interventions
    int nb_non_ambiguous = 0;
    // Number of interventions that we could reduce the time window for
    int nb_reductions = 0;
    // Number of interventions that can be done either in the morning or in the afternoon
    int nb_ambiguous = 0;
    // Only go through the interventions
    int number_interventions = instance.number_interventions;
    for (int i = 0; i < number_interventions; i++){
        Node& intervention = instance.nodes[i];
        // Is the intervention a long intervention?
        if (intervention.duration >= LONG_INTERVENTION){
            nb_non_ambiguous++;
            intervention.is_ambiguous = false;
            continue;
        }
        // Is it fully in the morning ?
        bool is_morning = intervention.end_window <= MID_DAY;
        // Is it fully in the afternoon ?
        bool is_afternoon = intervention.start_window >= MID_DAY;
        // Can it be done fully in the morning ?
        bool can_morning = intervention.start_window + intervention.duration <= MID_DAY;
        // Can it be done fully in the afternoon ?
        bool can_afternoon = intervention.end_window >= MID_DAY + intervention.duration;
        // If it is fully in the morning or fully in the afternoon, we do not need to do anything
        if (is_morning || is_afternoon){
            intervention.is_ambiguous = false;
            nb_non_ambiguous++;
        }
        // If it can be done in the morning but not in the afternoon
        else if (can_morning && !can_afternoon){
            intervention.end_window = MID_DAY;
            intervention.is_ambiguous = false;
            nb_reductions++;
        }
        // If it can be done in the afternoon but not in the morning
        else if (!can_morning && can_afternoon){
            intervention.start_window = MID_DAY;
            intervention.is_ambiguous = false;
            nb_reductions++;
        }
        // If it can be done in both the morning and the afternoon
        else if (can_morning && can_afternoon){
            intervention.is_ambiguous = true;
            nb_ambiguous++;
        }
    }

    // Print the informations
    if (verbose){
        cout << "Number of non-ambiguous interventions: " << nb_non_ambiguous << endl;
        cout << "Number of interventions that we could reduce the time window for: " << nb_reductions << endl;
        cout << "Number of ambiguous interventions: " << nb_ambiguous << endl;
        cout << "----------------------------------------" << endl;
    }

    return;
}


bool is_trivially_non_feasible(const Node &intervention, const Instance &instance, const std::vector<int> &available_vehicle){
    // First, we check if the time window is a-priori feasible
    if (intervention.start_window + intervention.duration > intervention.end_window){
        return true;
    }

    // We then want to make sure that none of the consumption exceed the vehicle capacities
    // First, we create a vector of the capacities - the maximum of the capacities of the available vehicles
    std::map<std::string, int> max_capacities;
    for (int v : available_vehicle){
        for (const auto& key : instance.capacities_labels){
            max_capacities[key] = std::max(max_capacities[key], instance.vehicles[v].capacities.at(key));
        }
    }

    // Then, we check if the intervention can be done with the maximum capacities
    for (const auto& key : instance.capacities_labels){
        if (intervention.quantities.at(key) > max_capacities[key]){
            return true;
        }
    }

    return false;
}


int max_a_priori_feasible_time(const Instance &instance, bool verbose){
    // First step is building the list of available vehicles for each intervention
    std::vector<std::vector<int>> available_vehicles(instance.number_interventions);

    // Go through the vehicles
    for (int v = 0; v < instance.number_vehicles; v++){
        // Go through the interventions
        for (int i : instance.vehicles[v].interventions){
            available_vehicles[i].push_back(v);
        }
    }

    int total_time = 0;
    int total_count = 0;
    // Go through the interventions
    for (int i = 0; i < instance.number_interventions; i++){
        // If the intervention is trivially non feasible, we skip it
        if (is_trivially_non_feasible(instance.nodes[i], instance, available_vehicles[i])){
            continue;
        }
        total_time += instance.nodes[i].duration;
        total_count++;
    }
    if (verbose){
        cout << "Total time of the a-priori feasible interventions: " << total_time << endl;
        cout << "Number of a-priori feasible interventions: " << total_count << endl;
        cout << "----------------------------------------" << endl;
    }

    return total_time;
}