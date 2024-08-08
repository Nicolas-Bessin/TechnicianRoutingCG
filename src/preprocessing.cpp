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
        if (intervention.duration > LONG_INTERVENTION){
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