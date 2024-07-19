#include "solution.h"
#include <math.h>

// Checks if two routes are equal
// That is, if they have :
// - the same vehicle_id
// - the same sequence vector
// - the same start times
bool operator==(const Route& lhs, const Route& rhs){
    // Check the vehicle_id
    if (lhs.vehicle_id != rhs.vehicle_id){
        return false;
    }
    //Check the length of the id_sequence
    if (lhs.id_sequence.size() != rhs.id_sequence.size()){
        return false;
    }
    // Check the sequences themselves
    for (int i = 0; i < lhs.id_sequence.size(); i++){
        if (lhs.id_sequence[i] != rhs.id_sequence[i]){
            return false;
        }
    }
    // Check the start times
    for (int i = 0; i < lhs.start_times.size(); i++){
        if (fabs(lhs.start_times[i] - rhs.start_times[i]) > 0.0001){
            return false;
        }
    }

    return true;
}