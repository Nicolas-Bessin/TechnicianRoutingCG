/*
    // Load the test_instance.txt file
    std::string filename = "../test_instance.txt";
    vector<int> alphas = {0, 0, 0, 0, 0, 0};
    Solver solver;
    solver.readProblem(filename);
    solver.setupAlgorithms();
    solver.solve(); obj->setArcCost(1, 4, 2);
    obj->setArcCost(2, 1, 1);
    obj->setArcCost(2, 3, 5);
    obj->setArcCost(2, 4, 12);
    obj->setArcCost(3, 4, 10);
    obj->setArcCost(3, 5, 1);
    obj->setArcCost(4, 5, 2);

    obj->setNodeCost(1, 0);
    // Create the resource
    TimeWindow* tw = new TimeWindow();
    tw->initData(true, 6);
    // Edge consumptions
    tw->setArcCost(0, 1, 10);
    tw->setArcCost(0, 2, 3);
    tw->setArcCost(1, 3, 1);
    tw->setArcCost(1, 4, 3);
    tw->setArcCost(2, 1, 2);
    tw->setArcCost(2, 3, 7);
    tw->setArcCost(2, 4, 3);
    tw->setArcCost(3, 4, 1);
    tw->setArcCost(3, 5, 7);
    tw->setArcCost(4, 5, 2);
    // Node bounds
    tw->setNodeBound(6, 0, 0, 20);
    tw->setNodeBound(6, 1, 0, 20);
    tw->setNodeBound(6, 2, 0, 20);
    tw->setNodeBound(6, 3, 0, 20);
    tw->setNodeBound(6, 4, 0, 20);
    tw->setNodeBound(6, 5, 0, 20);

    tw->init(0, 5);
    // Set the resource
    problem->setRes(tw);
    // Set the objective
    problem->setObjective(obj);

    // Solve the problem
    Solver solver2;
    solver2.setCustomProblem(*problem);
    solver2.setupAlgorithms();
    solver2.solve();
    solver2.printBestSolution();
}

/*
    solver.printBestSolution();

    cout << "----------------------------------------------------------------" << endl;

    // What if we create the same problem by hand ?
    Problem* problem = new Problem("test", 6, 0, 5, 1, 0, 0, 1);
    problem->initProblem();
    Resource* obj = problem->getObj();
    obj->initData(true, 6);
    // Set the arcs in the graph
    problem->setNetworkArc(0, 1);
    problem->setNetworkArc(0, 2);
    problem->setNetworkArc(1, 3);
    problem->setNetworkArc(1, 4);
    problem->setNetworkArc(2, 1);
    problem->setNetworkArc(2, 3);
    problem->setNetworkArc(2, 4);
    problem->setNetworkArc(3, 4);
    problem->setNetworkArc(3, 5);
    problem->setNetworkArc(4, 5);
    // Set the costs
    obj->setArcCost(0, 1, 1);
    obj->setArcCost(0, 2, 10);
    obj->setArcCost(1, 3, 1);
    obj->setArcCost(1, 4, 2);
    obj->setArcCost(2, 1, 1);
    obj->setArcCost(2, 3, 5);
    obj->setArcCost(2, 4, 12);
    obj->setArcCost(3, 4, 10);
    obj->setArcCost(3, 5, 1);
    obj->setArcCost(4, 5, 2);

    obj->setNodeCost(1, 0);
    // Create the resource
    TimeWindow* tw = new TimeWindow();
    tw->initData(true, 6);
    // Edge consumptions
    tw->setArcCost(0, 1, 10);
    tw->setArcCost(0, 2, 3);
    tw->setArcCost(1, 3, 1);
    tw->setArcCost(1, 4, 3);
    tw->setArcCost(2, 1, 2);
    tw->setArcCost(2, 3, 7);
    tw->setArcCost(2, 4, 3);
    tw->setArcCost(3, 4, 1);
    tw->setArcCost(3, 5, 7);
    tw->setArcCost(4, 5, 2);
    // Node bounds
    tw->setNodeBound(6, 0, 0, 20);
    tw->setNodeBound(6, 1, 0, 20);
    tw->setNodeBound(6, 2, 0, 20);
    tw->setNodeBound(6, 3, 0, 20);
    tw->setNodeBound(6, 4, 0, 20);
    tw->setNodeBound(6, 5, 0, 20);

    tw->init(0, 5);
    // Set the resource
    problem->setRes(tw);
    // Set the objective
    problem->setObjective(obj);

    // Solve the problem
    Solver solver2;
    solver2.setCustomProblem(*problem);
    solver2.setupAlgorithms();
    solver2.solve();
    solver2.printBestSolution();
}

*/