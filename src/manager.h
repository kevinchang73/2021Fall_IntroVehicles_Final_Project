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
    void writeOutput(fstream&);
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
    // For First-Come-First-Serve
    void _extractInitialSolutionFCFS(bool print = PRINT);
    void _determineChangePointsFCFS(Vehicle* v, bool print = PRINT);

    // Compute Longest Passing Time
    double _computeCost(bool print = PRINT);
    void   _topologicalSort(vector<Node*>&, bool print = PRINT);
    void   _topologicalSortUtil(Node*, bool [], stack<Node*>&);
    
    // Solution
    void _updateBestSolution();
    void _restoreBestSolution();  
    
    // Simulated Annealing
    void _simulatedAnnealing(bool print = PRINT);
    void _findGreatStartPoint(bool print = PRINT);
    void _permutation(bool print = PRINT);
    void _changeOnePath(Vehicle*&, bool print = PRINT);
    void _changeOnePriority(Lane*&, int&, bool print = PRINT);
    bool _isPriorityChangingValid(Lane*, int, bool print = PRINT);
    void _modifyOrder(Lane*, int, bool print = PRINT);
    void _changeOneCrossLaneConstraint(bool print = PRINT);

    // Output
    static bool _compareForOutput(const Node* a, const Node* b){
        if(a->_lane == -1) return true;
        if(b->_lane == -1) return false;
        if(a->_vehicle->_id < b->_vehicle->_id){
            return true;
        }
        if(a->_vehicle->_id == b->_vehicle->_id){
            if(a->_point < b->_point){
                return true;
            }
        }
        return false;
    }
};

#endif //MANAGER_H
