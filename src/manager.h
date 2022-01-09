#ifndef MANAGER_H
#define MANAGER_H

#include <fstream>
#include <vector>
#include <map>
#include <unordered_map>
#include <set>
#include <stack>
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
    vector<Lane*>    _lanes;
    //vector< vector<Point*> >  _points;
    //vector<Node*>    _nodes;
    //vector<Edge*>    _edges;
    int _numDiscretePoints;
    int _numLanes;
    int _numExitLanes;
    int _numRemainLanes;
    int _numVehicles;
    double _tST, _tCT, _tSS, _tCS;
    Node* _psuedoSource = 0;
    set<Node*> _extractedNodes;
    vector<Node*> _bestSolution;

    // Initialization
    void _initialize();
    void _initializeNodes(Vehicle*, bool print = PRINT);
    void _initializePaths(Vehicle*, bool print = PRINT);
    void _initializeLanePriority(bool print = PRINT);
    void _initializeSameLaneConstraint(Lane*, bool print = PRINT);
    void _initializeCrossLaneConstraint(bool print = PRINT);
    static bool _comparePriorityByEat(const Vehicle* a, const Vehicle* b){
        return a->_earliestAT < b->_earliestAT;
    }
    bool _addCrossLaneConstraint(Node* inn1, Node* inn2, Node* out1, Node* out2, bool print = PRINT);

    // Extracting Solution Graph
    void _extractInitialSolution(bool print = PRINT);
    void _determineChangePoints(Vehicle* v, bool print = PRINT);
    void _enableTrajectory(Vehicle* v, bool print = PRINT);
    void _disableTrajectory(Vehicle* v, bool print = PRINT);
    void _addPsuedoSource(bool print = PRINT);
    void _printExtractedGraph();

    // Compute Longest Passing Time
    double _computeCost(bool print = PRINT);
    void   _topologicalSort(vector<Node*>&, bool print = PRINT);
    void   _topologicalSortUtil(Node*, bool [], stack<Node*>&);
    void   _updateBestSolution();
    
    //permutation
};

#endif //MANAGER_H
