#include "src/parser.h"
#include "src/cg_solver.h"
#include <iostream>


using namespace std;

int main(int, char**){
    cout << "Technician Routing Problem using Column Generation" << endl;
    string filename = "../data/instance_1.json";
    Instance instance = parse_file(filename);

    cg_solver(instance, 1000);
}

