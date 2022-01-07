#ifndef MANAGER_H
#define MANAGER_H

#include <fstream>
#include <vector>
#include <map>
#include <unordered_map>
#include <set>
#include "item.h"
using namespace std;

#define PRINT false

class Manager
{
public:
    Manager() {}
    ~Manager() {}
    void parseInput(fstream&);
    void plan();
    void writeOutput(fstream&) {};
private:
    // Members
    vector<Vehicle*> _vehicles;
    vector< vector<Point*> >  _points;
    vector<Node*>    _nodes;
    vector<Edge*>    _edges;
    int _numDiscretePoints;
    int _numLanes;
    int _numExitLanes;
    int _numRemianLanes;
    int _numVehicles;
    double _tST, _tCT, _tSS, _tCS;

    // Initialization
    void _initialize();
    void _initializeNodes(Vehicle*, bool print = PRINT);
    void _initializePaths(Vehicle*, bool print = PRINT);
};

#endif //MANAGER_H
