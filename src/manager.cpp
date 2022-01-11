#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <cassert>
#include <vector>
#include <cmath>
#include <map>
#include <algorithm>
#include <cstdlib>
#include <stack>
#include "manager.h"
#include "item.h"
using namespace std;

void Manager::parseInput(fstream& input) {
    input >> _tST;
    input >> _tCT;
    input >> _tSS;
    input >> _tCS;
    input >> _numDiscretePoints;
    input >> _numLanes;
    input >> _numExitLanes;
    input >> _numVehicles;
    _numRemainLanes = _numLanes - _numExitLanes;
    assert(_numDiscretePoints > max(_numExitLanes, _numRemainLanes));
    for(int i=0; i<_numVehicles; ++i){
        int id, lane;
        bool exit;
        double eat;
        input >> id >> lane >> exit >> eat;
        Vehicle* v = new Vehicle(id, lane, exit, eat);
        if(exit){
            if(lane < _numRemainLanes){
                v->_type = Trajectory_type::ON_TO_OFF;
            }
            else{
                v->_type = Trajectory_type::OFF_TO_OFF;
            }
        }
        else{
            if(lane < _numRemainLanes){
                v->_type = Trajectory_type::ON_TO_ON;
            }
            else{
                v->_type = Trajectory_type::OFF_TO_ON;
            }
        }
        _vehicles.push_back(v);
    }
    if(PRINT){
        for(int i=0; i<_numVehicles; ++i){
            _vehicles[i]->print();
        }
    }
}
void Manager::plan() {
    double bestCost = numeric_limits<double>::max();
    double currCost;
    cout << "Initialization...\n";
    _initialize();
    cout << "Extract initial solution...\n";
    _extractInitialSolution(false);
    //_extractInitialSolutionFCFS(false);
    cout << "Compute initial cost...";
    currCost = _computeCost(false);
    if(currCost < bestCost){
        bestCost = min(currCost, bestCost);
        _updateBestSolution();
    }
    cout << "Initial Cost = " << bestCost << "\n";
    cout << "Perform simulated annealing...\n";
    //_findGreatStartPoint(true);
    _simulatedAnnealing(true);
}
void Manager::writeOutput(fstream& out){
    sort(_bestSolution.begin(), _bestSolution.end(), _compareForOutput);
    int idx = 1;
    for(int i=0; i<_numVehicles; ++i){
        out << _vehicles[i]->_id << " ";
        for(int k=0; k<_numDiscretePoints; ++k){
            out << "(" << _bestSolution[idx]->_vehicle->_id << "," << _bestSolution[idx]->_lane << "," << _bestSolution[idx]->_bestArrivalTime << ") ";
            ++idx;
        }
        out << "\n";
    }
}

void Manager::_initialize() {
    // discrete points
    for(int i=0; i<_numLanes; ++i){
        Lane* lane = new Lane(i);
        //vector<Point*> lane;
        lane->_points.reserve(_numDiscretePoints);
        for(int j=0; j<_numDiscretePoints; ++j){
            Point* point = new Point(i, j);
            lane->_points.push_back(point);
        }
        _lanes.push_back(lane);
    }
    // graph nodes and path edges
    for(int i=0; i<_numVehicles; ++i){
        _initializeNodes(_vehicles[i]);
        _initializePaths(_vehicles[i]);
    }
    // passing order
    _initializeLanePriority();
    // constraint edges
    for(int h=0; h<_numLanes; ++h){
        _initializeSameLaneConstraint(_lanes[h]);
    }
    _initializeCrossLaneConstraint();
}
void Manager::_initializeNodes(Vehicle* v, bool print){
    if(print) cout << "Vehicle " << v->_id << " " << v->_type << "\n";
    if(v->_type == Trajectory_type::ON_TO_OFF){
        int firstExitLane = _numRemainLanes;
        int crossLane = firstExitLane - v->_incomingLane + 1;
        for(int h=v->_incomingLane; h<=firstExitLane; ++h){
            vector<Node*> laneOfNodes;
            for(int k=h-v->_incomingLane; k<_numDiscretePoints-crossLane+1+h-v->_incomingLane; ++k){
                Node* node = new Node(h, k, v);
                _lanes[h]->_points[k]->_vehicleID2Node[v->_id] = node;
                laneOfNodes.push_back(node);
                if(print) cout << "(" << h << "," << k << ") ";
                _lanes[h]->_passVehicles.insert(v->_id);
            }
            v->_nodes.push_back(laneOfNodes);
        }
        for(int h=firstExitLane+1; h<_numLanes; ++h){
            vector<Node*> laneOfNodes;
            for(int k=crossLane+h-(firstExitLane+1); k<_numDiscretePoints; ++k){
                Node* node = new Node(h, k, v);
                _lanes[h]->_points[k]->_vehicleID2Node[v->_id] = node;
                laneOfNodes.push_back(node);
                if(print) cout << "(" << h << "," << k << ") ";
                _lanes[h]->_passVehicles.insert(v->_id);
            }
            v->_nodes.push_back(laneOfNodes);
        }
    }
    else if(v->_type == Trajectory_type::OFF_TO_OFF){
        for(int h=v->_incomingLane; h<_numLanes; ++h){
            vector<Node*> laneOfNodes;
            for(int k=h-v->_incomingLane; k<_numDiscretePoints; ++k){
                Node* node = new Node(h, k, v);
                _lanes[h]->_points[k]->_vehicleID2Node[v->_id] = node;
                laneOfNodes.push_back(node);
                if(print) cout << "(" << h << "," << k << ") ";
                _lanes[h]->_passVehicles.insert(v->_id);
            }
            v->_nodes.push_back(laneOfNodes);
        }
    }
    else if(v->_type == Trajectory_type::ON_TO_ON){
        int h = v->_incomingLane;
        vector<Node*> laneOfNodes;
        for(int k=0; k<_numDiscretePoints; ++k){
            Node* node = new Node(v->_incomingLane, k, v);
            _lanes[h]->_points[k]->_vehicleID2Node[v->_id] = node;
            laneOfNodes.push_back(node);
            if(print) cout << "(" << h << "," << k << ") ";
            _lanes[h]->_passVehicles.insert(v->_id);
        }
        v->_nodes.push_back(laneOfNodes);
    }
    else if(v->_type == Trajectory_type::OFF_TO_ON){
        int firstRemainLane = _numRemainLanes - 1;
        int crossLane = v->_incomingLane - firstRemainLane + 1;
        for(int h=v->_incomingLane; h>=firstRemainLane; --h){
            vector<Node*> laneOfNodes;
            for(int k=v->_incomingLane-h; k<_numDiscretePoints-crossLane+1+v->_incomingLane-h; ++k){
                //int pointId = v->_incomingLane - h + k;
                Node* node = new Node(h, k, v);
                _lanes[h]->_points[k]->_vehicleID2Node[v->_id] = node;
                laneOfNodes.push_back(node);
                if(print) cout << "(" << h << "," << k << ") ";
                _lanes[h]->_passVehicles.insert(v->_id);
            }
            v->_nodes.push_back(laneOfNodes);
        }
        for(int h=firstRemainLane-1; h>=0; --h){
            vector<Node*> laneOfNodes;
            for(int k=v->_incomingLane-h; k<_numDiscretePoints; ++k){
                Node* node = new Node(h, k, v);
                _lanes[h]->_points[k]->_vehicleID2Node[v->_id] = node;
                laneOfNodes.push_back(node);
                if(print) cout << "(" << h << "," << k << ") ";
                _lanes[h]->_passVehicles.insert(v->_id);
            }
            v->_nodes.push_back(laneOfNodes);
        }
    }
    if(print) cout << "\n";
}
void Manager::_initializePaths(Vehicle* v, bool print){
    if(print) cout << "Vehicle " << v->_id << "\n";
    if(v->_type == Trajectory_type::ON_TO_OFF || v->_type == Trajectory_type::OFF_TO_OFF){
        for(int h=v->_incomingLane; h<_numLanes; ++h){
            for(int k=0; k<_numDiscretePoints-1; ++k){
                if(_lanes[h]->_points[k]->_vehicleID2Node.find(v->_id) != _lanes[h]->_points[k]->_vehicleID2Node.end()){
                    //cross-lane edges
                    if(h<_numLanes-1){
                        assert(_lanes[h+1]->_points[k+1]->_vehicleID2Node.find(v->_id) != _lanes[h+1]->_points[k+1]->_vehicleID2Node.end());
                        Node* node_u = _lanes[h]->_points[k]->_vehicleID2Node[v->_id];
                        Node* node_v = _lanes[h+1]->_points[k+1]->_vehicleID2Node[v->_id];
                        Edge* edge = new Edge(node_u, node_v, Edge_type::CROSS_LANE_PATH, _tCT);
                        node_u->_outCrossLaneEdge = edge;
                        node_v->_inCrossLaneEdge = edge;
                        if(print) cout << "(" << node_u->_vehicle->_id << " " << node_u->_lane << " " << node_u->_point << "," << node_v->_vehicle->_id << " " << node_v->_lane << " " << node_v->_point<< ") ";
                    }
                    //same-lane edges
                    if(_lanes[h]->_points[k+1]->_vehicleID2Node.find(v->_id) != _lanes[h]->_points[k+1]->_vehicleID2Node.end()){
                        Node* node_u = _lanes[h]->_points[k]->_vehicleID2Node[v->_id];
                        Node* node_v = _lanes[h]->_points[k+1]->_vehicleID2Node[v->_id];
                        Edge* edge = new Edge(node_u, node_v, Edge_type::SAME_LANE_PATH, _tST);
                        node_u->_outSameLaneEdge = edge;
                        node_v->_inSameLaneEdge = edge;
                        if(print) cout << "(" << node_u->_vehicle->_id << " " << node_u->_lane << " " << node_u->_point << "," << node_v->_vehicle->_id << " " << node_v->_lane << " " << node_v->_point<< ") ";
                    }
                }
            }
        }
    }
    else if(v->_type == Trajectory_type::ON_TO_ON){
        int h = v->_incomingLane;
        for(int k=0; k<_numDiscretePoints-1; ++k){
            assert(_lanes[h]->_points[k]->_vehicleID2Node.find(v->_id) != _lanes[h]->_points[k]->_vehicleID2Node.end());
            assert(_lanes[h]->_points[k+1]->_vehicleID2Node.find(v->_id) != _lanes[h]->_points[k+1]->_vehicleID2Node.end());
            Node* node_u = _lanes[h]->_points[k]->_vehicleID2Node[v->_id];
            Node* node_v = _lanes[h]->_points[k+1]->_vehicleID2Node[v->_id];
            Edge* edge = new Edge(node_u, node_v, Edge_type::SAME_LANE_PATH, _tST);
            node_u->_outSameLaneEdge = edge;
            node_v->_inSameLaneEdge = edge;
            if(print) cout << "(" << node_u->_vehicle->_id << " " << node_u->_lane << " " << node_u->_point << "," << node_v->_vehicle->_id << " " << node_v->_lane << " " << node_v->_point<< ") ";
        }
    }
    else if(v->_type == Trajectory_type::OFF_TO_ON){
        for(int h=v->_incomingLane; h>=0; --h){
            for(int k=0; k<_numDiscretePoints-1; ++k){
                if(_lanes[h]->_points[k]->_vehicleID2Node.find(v->_id) != _lanes[h]->_points[k]->_vehicleID2Node.end()){
                    //cross-lane edges
                    if(h>0){
                        assert(_lanes[h-1]->_points[k+1]->_vehicleID2Node.find(v->_id) != _lanes[h-1]->_points[k+1]->_vehicleID2Node.end());
                        Node* node_u = _lanes[h]->_points[k]->_vehicleID2Node[v->_id];
                        Node* node_v = _lanes[h-1]->_points[k+1]->_vehicleID2Node[v->_id];
                        Edge* edge = new Edge(node_u, node_v, Edge_type::CROSS_LANE_PATH, _tCT);
                        node_u->_outCrossLaneEdge = edge;
                        node_v->_inCrossLaneEdge = edge;
                        if(print) cout << "(" << node_u->_vehicle->_id << " " << node_u->_lane << " " << node_u->_point << "," << node_v->_vehicle->_id << " " << node_v->_lane << " " << node_v->_point<< ") ";
                    }
                    //same-lane edges
                    if(_lanes[h]->_points[k+1]->_vehicleID2Node.find(v->_id) != _lanes[h]->_points[k+1]->_vehicleID2Node.end()){
                        Node* node_u = _lanes[h]->_points[k]->_vehicleID2Node[v->_id];
                        Node* node_v = _lanes[h]->_points[k+1]->_vehicleID2Node[v->_id];
                        Edge* edge = new Edge(node_u, node_v, Edge_type::SAME_LANE_PATH, _tST);
                        node_u->_outSameLaneEdge = edge;
                        node_v->_inSameLaneEdge = edge;
                        if(print) cout << "(" << node_u->_vehicle->_id << " " << node_u->_lane << " " << node_u->_point << "," << node_v->_vehicle->_id << " " << node_v->_lane << " " << node_v->_point<< ") ";
                    }
                }
            }
        }
    }
    if(print) cout << "\n";
}
void Manager::_initializeLanePriority(bool print){
    vector<Vehicle*> priority;
    for(int i=0; i<_numVehicles; ++i){
        priority.push_back(_vehicles[i]);
    }
    sort(priority.begin(), priority.end(), _comparePriorityByEat);
    for(int h=0; h<_numLanes; ++h){
        Lane* currLane = _lanes[h];
        for(int i=0; i<_numVehicles; ++i){
            int id = priority[i]->_id;
            if(currLane->_passVehicles.find(id) != currLane->_passVehicles.end()){
                currLane->_order.push_back(id);
                for(int k=0; k<_numDiscretePoints; ++k){
                    Point* point = currLane->_points[k];
                    if(point->_vehicleID2Node.find(id) != point->_vehicleID2Node.end()){
                        point->_vehicleID2Node[id]->_priority = currLane->_order.size() - 1;
                    }
                }
            }
        }
    }
    if(print){
        for(int h=0; h<_numLanes; ++h){
            cout << "Lane " << h << ":\n";
            Lane* currLane = _lanes[h];
            cout << "Passing vehicles: ";
            for(auto ite = currLane->_passVehicles.begin(); ite != currLane->_passVehicles.end(); ++ite){
                cout << *ite << " ";
            }
            cout << "\n";
            cout << "Priority:";
            for(size_t j=0; j<currLane->_order.size(); ++j){
                cout << currLane->_order[j] << " ";
            }
            cout << "\n";
        }
    }
}
void Manager::_initializeSameLaneConstraint(Lane* lane, bool print){
    if(print) cout << "Lane " << lane->_idx << ":\n";
    for(int k=0; k<_numDiscretePoints; ++k){
        if(print) cout << "Point " << k << ":\n";
        Point* p = lane->_points[k];
        for(auto u = p->_vehicleID2Node.begin(); u != p->_vehicleID2Node.end(); ++u){
            for(auto v = u; v!=p->_vehicleID2Node.end(); ++v){
                if(u != v){
                    Node* node_u = u->second;
                    Node* node_v = v->second;
                    assert(node_u!=node_v);
                    Edge* edge = new Edge(node_u, node_v, Edge_type::SAME_LANE_CONSTRAINT, _tSS);
                    if(node_v->_priority<node_u->_priority){
                        edge->reverse();
                    }
                    edge->_u->_outSameLaneConsEdges.push_back(edge);
                    edge->_v->_inSameLaneConsEdges.push_back(edge);
                    if(print) cout << "(" << edge->_u->_vehicle->_id << " " << edge->_u->_lane << " " << edge->_u->_point << "," << edge->_v->_vehicle->_id << " " << edge->_v->_lane << " " << edge->_v->_point<< ") ";
                }
            }
        }
        if(print) cout << "\n";
    }
    if(print) cout << "\n";
}
void Manager::_initializeCrossLaneConstraint(bool print){
    if(print) cout << "Cross lane contraints:\n";
    for(int h=0; h<_numLanes; ++h){
        for(int g=h+1; g<_numLanes; ++g){
            for(int k=0; k<_numDiscretePoints-1; ++k){
                Point* inn1 = _lanes[h]->_points[k];
                Point* out1 = _lanes[g]->_points[k];
                Point* inn2 = _lanes[h]->_points[k+1];
                Point* out2 = _lanes[g]->_points[k+1];
                for(int i=0; i<_numVehicles; ++i){
                    for(int j=0; j<_numVehicles; ++j){
                        Vehicle* v1 = _vehicles[i];
                        Vehicle* v2 = _vehicles[j];
                        if(inn1->_vehicleID2Node.find(v1->_id) != inn1->_vehicleID2Node.end() &&
                           inn2->_vehicleID2Node.find(v2->_id) != inn2->_vehicleID2Node.end() &&
                           out1->_vehicleID2Node.find(v2->_id) != out1->_vehicleID2Node.end() &&
                           out2->_vehicleID2Node.find(v1->_id) != out2->_vehicleID2Node.end()){
                            _addCrossLaneConstraint(inn1->_vehicleID2Node[v1->_id], inn2->_vehicleID2Node[v2->_id], out1->_vehicleID2Node[v2->_id], out2->_vehicleID2Node[v1->_id], print);
                        }
                    }
                }
            }
        }
    }
    if(print) cout << "\n";
}
bool Manager::_addCrossLaneConstraint(Node* inn1, Node* inn2, Node* out1, Node* out2, bool print){
    //if(inn1->_outCrossLaneEdge && out1->_outCrossLaneEdge){ //initialized as inner vehicles are always prior to outer vehicles
    //    if(inn1->_outCrossLaneEdge->_v == out2 && out1->_outCrossLaneEdge->_v == inn2){
    //        Node* node_u = out2;
    //        Node* node_v = inn2;
    //        Edge* edge = new Edge(node_u, node_v, Edge_type::CROSS_LANE_CONSTRAINT, _tCS);
    //        node_u->_outCrossLaneConsEdges.push_back(edge);
    //        node_v->_inCrossLaneConsEdges.push_back(edge);
    //        if(print) cout << "(" << node_u->_vehicle->_id << " " << node_u->_lane << " " << node_u->_point << "," << node_v->_vehicle->_id << " " << node_v->_lane << " " << node_v->_point<< ") ";
    //        return true;
    //    }
    //}
    if(inn1->_outCrossLaneEdge && out1->_outCrossLaneEdge){ //initialized as the earliest arriving car has the highest priority
        if(inn1->_outCrossLaneEdge->_v == out2 && out1->_outCrossLaneEdge->_v == inn2){
            Node* node_u = (out2->_vehicle->_earliestAT < inn2->_vehicle->_earliestAT)? out2 : inn2;
            Node* node_v = (out2->_vehicle->_earliestAT < inn2->_vehicle->_earliestAT)? inn2 : out2;
            Edge* edge = new Edge(node_u, node_v, Edge_type::CROSS_LANE_CONSTRAINT, _tCS);
            node_u->_outCrossLaneConsEdges.push_back(edge);
            node_v->_inCrossLaneConsEdges.push_back(edge);
            if(print) cout << "(" << node_u->_vehicle->_id << " " << node_u->_lane << " " << node_u->_point << "," << node_v->_vehicle->_id << " " << node_v->_lane << " " << node_v->_point<< ") ";
            return true;
        }
    }
    return false;
}

void Manager::_extractInitialSolution(bool print){
    for(int i=0; i<_numVehicles; ++i){
        _determineChangePoints(_vehicles[i], print);
        _enableTrajectory(_vehicles[i], print);
    }
    _addPsuedoSource(print);
    if(print) _printExtractedGraph();
}
void Manager::_extractInitialSolutionFCFS(bool print){
    for(int i=0; i<_numVehicles; ++i){
        _determineChangePointsFCFS(_vehicles[i], print);
        _enableTrajectory(_vehicles[i], print);
    }
    _addPsuedoSource(print);
    if(print) _printExtractedGraph();
}
void Manager::_determineChangePoints(Vehicle* v, bool print){
    if(print) cout << "Vehicle " << v->_id << ":\n";
    v->_changePoints.clear();
    v->_changePoints.resize(0);
    if(v->_type == Trajectory_type::ON_TO_OFF){
        int curr = 0;
        for(int h=v->_incomingLane; h<_numRemainLanes; ++h){
            int idx = h - v->_incomingLane;
            int start = curr;
            int end = v->_nodes[idx][v->_nodes[idx].size()-1]->_point;
            int changePoint = rand() % (end - start + 1) + start;
            v->_changePoints.push_back(changePoint);
            curr = changePoint + 1;
        }
        for(int h=_numRemainLanes; (curr<_numDiscretePoints-1) && (h<_numLanes-1); ++h){
            double prob = (double)rand()/(RAND_MAX+1.0);
            if(prob < (double)1.0/(double)_numRemainLanes){
                break;
            }
            int start = curr;
            int end = _numDiscretePoints - 2;
            int changePoint = rand() % (end - start + 1) + start;
            v->_changePoints.push_back(changePoint);
            curr = changePoint + 1;
        }
        if(print){
            for(size_t k=0; k<v->_changePoints.size(); ++k){
                cout << "(" << k+v->_incomingLane << "," << v->_changePoints[k] << ") ";
            }
        }
    }
    else if(v->_type == Trajectory_type::OFF_TO_OFF){
        int curr = 0;
        for(int h=v->_incomingLane; (curr<_numDiscretePoints-1) && (h<_numLanes-1); ++h){
            double prob = (double)rand()/(RAND_MAX+1.0);
            if(prob < (double)1.0/(double)_numExitLanes){
                break;
            }
            int start = curr;
            int end = _numDiscretePoints - 2;
            int changePoint = rand() % (end - start + 1) + start;
            v->_changePoints.push_back(changePoint);
            curr = changePoint + 1;
        }
        if(print){
            for(size_t k=0; k<v->_changePoints.size(); ++k){
                cout << "(" << k+v->_incomingLane << "," << v->_changePoints[k] << ") ";
            }
        }
    }
    else if(v->_type == Trajectory_type::OFF_TO_ON){
        int curr = 0;
        for(int h=v->_incomingLane; h>=_numRemainLanes; --h){
            int idx = v->_incomingLane - h;
            int start = curr;
            int end = v->_nodes[idx][v->_nodes[idx].size()-1]->_point;
            int changePoint = rand() % (end - start + 1) + start;
            v->_changePoints.push_back(changePoint);
            curr = changePoint + 1;
        }
        for(int h=_numRemainLanes-1; (curr<_numDiscretePoints-1) && (h>0); --h){
            double prob = (double)rand()/(RAND_MAX+1.0);
            if(prob < (double)1.0/(double)_numRemainLanes){
                break;
            }
            int start = curr;
            int end = _numDiscretePoints - 2;
            int changePoint = rand() % (end - start + 1) + start;
            v->_changePoints.push_back(changePoint);
            curr = changePoint + 1;
        }
        if(print){
            for(size_t k=0; k<v->_changePoints.size(); ++k){
                cout << "(" << v->_incomingLane-k << "," << v->_changePoints[k] << ") ";
            }
        }
    }
    if(print) cout << "\n";
}
void Manager::_determineChangePointsFCFS(Vehicle* v, bool print){
    if(print) cout << "Vehicle " << v->_id << ":\n";
    v->_changePoints.clear();
    v->_changePoints.resize(0);
    if(v->_type == Trajectory_type::ON_TO_OFF){
        for(int h=v->_incomingLane; h<_numRemainLanes; ++h){
            int idx = h - v->_incomingLane;
            v->_changePoints.push_back(idx);
        }
        if(print){
            for(size_t k=0; k<v->_changePoints.size(); ++k){
                cout << "(" << k+v->_incomingLane << "," << v->_changePoints[k] << ") ";
            }
        }
    }
    else if(v->_type == Trajectory_type::OFF_TO_OFF){
        //no change
    }
    else if(v->_type == Trajectory_type::OFF_TO_ON){
        for(int h=v->_incomingLane; h>=_numRemainLanes; --h){
            int idx = v->_incomingLane - h;
            v->_changePoints.push_back(idx);
        }
        if(print){
            for(size_t k=0; k<v->_changePoints.size(); ++k){
                cout << "(" << v->_incomingLane-k << "," << v->_changePoints[k] << ") ";
            }
        }
    }
    if(print) cout << "\n";
}
void Manager::_enableTrajectory(Vehicle* v, bool print){
    Node* currNode = v->_nodes[0][0];
    currNode->_enable = true;
    _extractedNodes.insert(currNode);
    if(print){
        cout << "Enable (";
        currNode->print();
        cout << ")\n";
    }
    for(int i=0; i<(int)v->_changePoints.size(); ){
        if(currNode->_point == v->_changePoints[i]){
            currNode = currNode->_outCrossLaneEdge->_v;
            ++i;
        }
        else{
            currNode = currNode->_outSameLaneEdge->_v;
        }
        currNode->_enable = true;
        _extractedNodes.insert(currNode);
        if(print){
            cout << "Enable (";
            currNode->print();
            cout << ")\n";
        }
    }
    while(currNode->_point < _numDiscretePoints - 1){
        currNode = currNode->_outSameLaneEdge->_v;
        currNode->_enable = true;
        _extractedNodes.insert(currNode);
        if(print){
            cout << "Enable (";
            currNode->print();
            cout << ")\n";
        }
    }
}
void Manager::_disableTrajectory(Vehicle* v, bool print){
    for(size_t i=0; i<v->_nodes.size(); ++i){
        for(size_t j=0; j<v->_nodes[i].size(); ++j){
            if(v->_nodes[i][j]->_enable){
                v->_nodes[i][j]->_enable = false;
                _extractedNodes.erase(v->_nodes[i][j]);
            }
        }
    }
}
void Manager::_addPsuedoSource(bool print){
    _psuedoSource = new Node(-1,-1,0);
    _psuedoSource->_enable = true;
    _extractedNodes.insert(_psuedoSource);
    for(int i=0; i<_numVehicles; ++i){
        Node* node_u = _psuedoSource;
        Node* node_v = _vehicles[i]->_nodes[0][0];
        Edge* edge = new Edge(node_u, node_v, Edge_type::PSUEDO, _vehicles[i]->_earliestAT);
        node_u->_outSameLaneConsEdges.push_back(edge);
        node_v->_inPsuedoEdge = edge;
    }
    if(print){
        for(size_t i=0; i<_psuedoSource->_outSameLaneConsEdges.size(); ++i){
            cout << "Add psuedo edge ";
            _psuedoSource->_outSameLaneConsEdges[i]->print();
            cout << "\n";
        }
    }
}
void Manager::_printExtractedGraph(){
    cout << "Exracted Nodes:\n";
    for(auto ite = _extractedNodes.begin(); ite != _extractedNodes.end(); ++ite){
        cout << "(";
        (*ite)->print();
        cout << ") ";
    }
    cout << "\nSource:\n";
    for(size_t i=0; i<_psuedoSource->_outSameLaneConsEdges.size(); ++i){
        if(_psuedoSource->_outSameLaneConsEdges[i]->enable()){
            _psuedoSource->_outSameLaneConsEdges[i]->print();
        }
    }
    cout << "\n";
    for(int i=0; i<_numVehicles; ++i){
        cout << "Vehicle " << _vehicles[i]->_id << ":\n";
        for(size_t j=0; j<_vehicles[i]->_nodes.size(); ++j){
            for(size_t k=0; k<_vehicles[i]->_nodes[j].size(); ++k){
                Node* node = _vehicles[i]->_nodes[j][k];
                if(node->_outCrossLaneEdge && node->_outCrossLaneEdge->enable()){
                    node->_outCrossLaneEdge->print();
                }
                if(node->_outSameLaneEdge && node->_outSameLaneEdge->enable()){
                    node->_outSameLaneEdge->print();
                }
                for(size_t l=0; l<node->_outSameLaneConsEdges.size(); ++l){
                    if(node->_outSameLaneConsEdges[l]->enable()){
                        node->_outSameLaneConsEdges[l]->print();
                    }
                }
                for(size_t l=0; l<node->_outCrossLaneConsEdges.size(); ++l){
                    if(node->_outCrossLaneConsEdges[l]->enable()){
                        node->_outCrossLaneConsEdges[l]->print();
                    }
                }
            }
        }
        cout << "\n";
    }
}

double Manager::_computeCost(bool print){
    vector<Node*> topoOrder;
    _topologicalSort(topoOrder, print);
    topoOrder[0]->_arrivalTime = 0;
    for(size_t i=1; i<topoOrder.size(); ++i){
        topoOrder[i]->_arrivalTime = -numeric_limits<double>::max();
    }
    for(size_t j=0; j<topoOrder.size()-1; ++j){
        Node* node = topoOrder[j];
        double arrival = node->_arrivalTime;
        if(node->_outSameLaneEdge && node->_outSameLaneEdge->enable()){
            node->_outSameLaneEdge->_v->_arrivalTime = max(arrival + node->_outSameLaneEdge->_weight, node->_outSameLaneEdge->_v->_arrivalTime);
        }
        if(node->_outCrossLaneEdge && node->_outCrossLaneEdge->enable()){
            node->_outCrossLaneEdge->_v->_arrivalTime = max(arrival + node->_outCrossLaneEdge->_weight, node->_outCrossLaneEdge->_v->_arrivalTime);
        }
        for(size_t i=0; i<node->_outSameLaneConsEdges.size(); ++i){
            if(node->_outSameLaneConsEdges[i]->enable()){
                node->_outSameLaneConsEdges[i]->_v->_arrivalTime = max(arrival + node->_outSameLaneConsEdges[i]->_weight, node->_outSameLaneConsEdges[i]->_v->_arrivalTime);
            }
        }
        for(size_t i=0; i<node->_outCrossLaneConsEdges.size(); ++i){
            if(node->_outCrossLaneConsEdges[i]->enable()){
                node->_outCrossLaneConsEdges[i]->_v->_arrivalTime = max(arrival + node->_outCrossLaneConsEdges[i]->_weight, node->_outCrossLaneConsEdges[i]->_v->_arrivalTime);
            }
        }
    }
    double cost = 0;
    for(size_t i=0; i<topoOrder.size(); ++i){
        if(print){
            cout << "(";
            topoOrder[i]->print();
            cout << ") AT = ";
            cout << topoOrder[i]->_arrivalTime << "\n";
        }
        if(topoOrder[i]->_arrivalTime > cost){
            cost = topoOrder[i]->_arrivalTime;
            if(print){
                cout << "Update max.\n";
            }
        }
    }
    return cost;
}
void Manager::_topologicalSort(vector<Node*>& topo, bool print){
    stack<Node*> s;
    bool* visited = new bool[_extractedNodes.size()];
    Node** nodes = new Node*[_extractedNodes.size()];
    auto ite=_extractedNodes.begin();
    for(size_t i=0; ite!=_extractedNodes.end(); ++ite){
        visited[i] = false;
        nodes[i] = *ite;
        (*ite)->_extractedIdx = i;
        ++i;
    }
    for(size_t i=0; i<_extractedNodes.size(); ++i){
        if(visited[i] == false){
            _topologicalSortUtil(nodes[i], visited, s);
        }
    }
    if(print) cout << "Topological order:\n";
    while(!s.empty()){
        topo.push_back(s.top());
        if(print) {
            cout << s.top()->_extractedIdx << " (";
            s.top()->print();
            cout << ")\n";
        }
        s.pop();
    }
    delete visited;
}
void Manager::_topologicalSortUtil(Node* node, bool visited[], stack<Node*>& s){
    visited[node->_extractedIdx] = true;
    if(node->_outSameLaneEdge && node->_outSameLaneEdge->enable()){
        if(visited[node->_outSameLaneEdge->_v->_extractedIdx] == false){
            _topologicalSortUtil(node->_outSameLaneEdge->_v, visited, s);
        }
    }
    if(node->_outCrossLaneEdge && node->_outCrossLaneEdge->enable()){
        if(visited[node->_outCrossLaneEdge->_v->_extractedIdx] == false){
            _topologicalSortUtil(node->_outCrossLaneEdge->_v, visited, s);
        }
    }
    for(size_t i=0; i<node->_outSameLaneConsEdges.size(); ++i){
        if(node->_outSameLaneConsEdges[i]->enable() && visited[node->_outSameLaneConsEdges[i]->_v->_extractedIdx] == false){
            _topologicalSortUtil(node->_outSameLaneConsEdges[i]->_v, visited, s);
        }
    }
    for(size_t i=0; i<node->_outCrossLaneConsEdges.size(); ++i){
        if(node->_outCrossLaneConsEdges[i]->enable() && visited[node->_outCrossLaneConsEdges[i]->_v->_extractedIdx] == false){
            _topologicalSortUtil(node->_outCrossLaneConsEdges[i]->_v, visited, s);
        }
    }
    s.push(node);
}

void Manager::_updateBestSolution(){
    _bestSolution.clear();
    _bestSolution.resize(0);
    for(auto ite=_extractedNodes.begin(); ite!=_extractedNodes.end(); ++ite){
        _bestSolution.push_back(*ite);
        (*ite)->_bestArrivalTime = (*ite)->_arrivalTime;
        //cout << "Store node (";
        //(*ite)->print();
        //cout << ") arrival time =" << (*ite)->_bestArrivalTime << "\n";
    }
    for(int i=0; i<_numVehicles; ++i){
        _vehicles[i]->_bestChangePoints.clear();
        _vehicles[i]->_bestChangePoints.resize(0);
        for(size_t k=0; k<_vehicles[i]->_changePoints.size(); ++k){
            _vehicles[i]->_bestChangePoints.push_back(_vehicles[i]->_changePoints[k]);
        }
    }
}
void Manager::_restoreBestSolution(){
    for(int i=0; i<_numVehicles; ++i){
        _vehicles[i]->_changePoints.clear();
        _vehicles[i]->_changePoints.resize(0);
        for(size_t k=0; k<_vehicles[i]->_bestChangePoints.size(); ++k){
            _vehicles[i]->_changePoints.push_back(_vehicles[i]->_bestChangePoints[k]);
        }
        _disableTrajectory(_vehicles[i]);
        _enableTrajectory(_vehicles[i]);
    }
}

void Manager::_simulatedAnnealing(bool print){
    //_permutation(print);
    double cost = _computeCost();
    double bestCost = cost;
    for(int i=0; i<500; ++i){
        double prob = rand() / (RAND_MAX + 1.0);
        if(prob < 0.2){ //change path
            Vehicle* toChange;
            if(print) cout << "Change path. ";
            _changeOnePath(toChange);
            double new_cost = _computeCost();
            if(new_cost < bestCost){
                bestCost = new_cost;
                cost = new_cost;
                _updateBestSolution();
                if(print) cout << "Best cost = " << bestCost << ", New cost = " << new_cost << ". Update best cost.\n";
            }
            else if(new_cost < cost + 0.000001){
                cost = new_cost;
                if(print) cout << "Best cost = " << bestCost << ", New cost = " << new_cost << ". Accept.\n";
            }
            else{
                prob = rand() / (RAND_MAX + 1.0);
                if(prob < 0.2){
                    if(print) cout << "Best cost = " << bestCost << ", New cost = " << new_cost << ". Accept with probability.\n";
                }
                else{
                    if(print) cout << "Best cost = " << bestCost << ", New cost = " << new_cost << ". Reject.\n";
                    toChange->_changePoints.clear();
                    toChange->_changePoints.resize(0);
                    for(size_t k=0; k<toChange->_prevChangePoints.size(); ++k){
                        toChange->_changePoints.push_back(toChange->_prevChangePoints[k]);
                    }
                }
            }
        }
        else{ //change priority
            if(print) cout << "Change prio. ";
            Lane* toChange;
            int idx;
            _changeOnePriority(toChange, idx);
            double new_cost = _computeCost();
            if(new_cost < bestCost){
                bestCost = new_cost;
                cost = new_cost;
                _updateBestSolution();
                if(print) cout << "Best cost = " << bestCost << ", New cost = " << new_cost << ". Update best cost.\n";
            }
            else if(new_cost < cost + 0.000001){
                cost = new_cost;
                if(print) cout << "Best cost = " << bestCost << ", New cost = " << new_cost << ". Accept.\n";
            }
            else{
                prob = rand() / (RAND_MAX + 1.0);
                if(prob < 0.2){
                    if(print) cout << "Best cost = " << bestCost << ", New cost = " << new_cost << ". Accept with probability.\n";
                }
                else{
                    if(print) cout << "Best cost = " << bestCost << ", New cost = " << new_cost << ". Reject.\n";
                    _modifyOrder(toChange, idx);
                }
            }
        }
    }
}
void Manager::_findGreatStartPoint(bool print){
    double bestCost = numeric_limits<double>::max();
    double currCost;
    for(int i=0; i<30; ++i){
        Vehicle* dummy;
        _changeOnePath(dummy, print);
        currCost = _computeCost();
        if(print) cout << "Curr. cost = " << currCost << " best cost = " << bestCost << "\n";
        if(currCost < bestCost){
            _updateBestSolution();
            bestCost = currCost;
            //if(print) _printExtractedGraph();
        }
    }
    if(print) cout << "Restore:\n";
    _restoreBestSolution();
    //if(print) _printExtractedGraph();
    currCost = _computeCost();
    if(print) cout << currCost << "\n";
}
void Manager::_permutation(bool print){
    double cost;
    for(int i=0; i<5; ++i){
        //_changeOnePriority(print);
        //cost = _computeCost();
        //cout << "cost = " << cost << "\n";
    }
}
void Manager::_changeOnePath(Vehicle*& change, bool print){
    Vehicle* v;
    int id;
    do{
        id = rand()%(_numVehicles);
    }while(_vehicles[id]->_type == Trajectory_type::ON_TO_ON);
    v = _vehicles[id];
    v->_prevChangePoints.clear();
    v->_prevChangePoints.resize(0);
    for(size_t k=0; k<v->_changePoints.size(); ++k){
        v->_prevChangePoints.push_back(v->_changePoints[k]);
    }
    _determineChangePoints(v, print);
    _disableTrajectory(v);
    _enableTrajectory(v);
    if(print) _printExtractedGraph();
    change = v;
}
void Manager::_changeOnePriority(Lane*& change, int& idx, bool print){
    Lane* lane;
    int laneId;
    do{
        laneId = rand()%(_numLanes);
    }while(_lanes[laneId]->_passVehicles.size()<=1);
    lane = _lanes[laneId];
    int changeIdx;
    do{
        changeIdx = rand()%(lane->_order.size()-1);
    }while(!_isPriorityChangingValid(lane, changeIdx, print));
    if(print) cout << "Change lane " << laneId << ", idx = " << changeIdx << "\n";
    _modifyOrder(lane, changeIdx, print);
    change = lane;
    idx = changeIdx;
}
bool Manager::_isPriorityChangingValid(Lane* lane, int changeIdx, bool print){
    Vehicle* v1 = _vehicles[lane->_order[changeIdx]];
    Vehicle* v2 = _vehicles[lane->_order[changeIdx+1]];
    if(v1->_incomingLane == v2->_incomingLane){
        if(v1->_incomingLane == lane->_idx){
            assert(v1->_earliestAT <= v2->_earliestAT);
            if(print) cout << "Conflict " << v1->_id << " " << v2->_id << "\n";
            return false;
        }
        else{
            //May or may not change
        }
    }
    for(int k=0; k<_numDiscretePoints; ++k){
        Point* point = lane->_points[k];
        if(point->_vehicleID2Node.find(v1->_id) != point->_vehicleID2Node.end() && 
           point->_vehicleID2Node.find(v2->_id) != point->_vehicleID2Node.end()){
            Node* n1 = point->_vehicleID2Node[v1->_id];
            Node* n2 = point->_vehicleID2Node[v2->_id];
            set<Node*> checkSet;
            for(size_t i=0; i<n1->_outCrossLaneConsEdges.size(); ++i){
                checkSet.insert(n1->_outCrossLaneConsEdges[i]->_v);
            }
            for(size_t i=0; i<n2->_inCrossLaneConsEdges.size(); ++i){
                if(checkSet.find(n2->_inCrossLaneConsEdges[i]->_u) != checkSet.end()){
                    if(print){
                        cout << "Conflict ";
                        n1->print(); cout << " ";
                        n2->print(); cout << "\n";
                    }
                    return false;
                }
            }
        }
    }
    return true;
}
void Manager::_modifyOrder(Lane* lane, int idx, bool print){ //modify lane->_order and the same lane constraint edges
    Vehicle* v1 = _vehicles[lane->_order[idx]];
    Vehicle* v2 = _vehicles[lane->_order[idx+1]];
    for(int k=0; k<_numDiscretePoints; ++k){
        Point* point = lane->_points[k];
        if(point->_vehicleID2Node.find(v1->_id) != point->_vehicleID2Node.end() && 
           point->_vehicleID2Node.find(v2->_id) != point->_vehicleID2Node.end()){
            Node* n1 = point->_vehicleID2Node[v1->_id];
            Node* n2 = point->_vehicleID2Node[v2->_id];
            Edge* discard = 0;
            for(size_t i=0; i<n1->_outSameLaneConsEdges.size(); ++i){
                if(n1->_outSameLaneConsEdges[i]->_v == n2){
                    discard = n1->_outSameLaneConsEdges[i];
                    n1->_outSameLaneConsEdges.erase(n1->_outSameLaneConsEdges.begin()+i);
                    break;
                }
            }
            for(size_t i=0; i<n2->_inSameLaneConsEdges.size(); ++i){
                if(n2->_inSameLaneConsEdges[i]->_u == n1){
                    n2->_inSameLaneConsEdges.erase(n2->_inSameLaneConsEdges.begin()+i);
                    break;
                }
            }
            assert(discard != 0);
            delete discard;
            Edge* new_edge = new Edge(n2, n1, Edge_type::SAME_LANE_CONSTRAINT, _tSS);
            n2->_outSameLaneConsEdges.push_back(new_edge);
            n1->_inSameLaneConsEdges.push_back(new_edge);
        }
    }
    int temp = lane->_order[idx];
    lane->_order[idx] = lane->_order[idx+1];
    lane->_order[idx+1] = temp;
    if(print){
        _printExtractedGraph();
    }
}
void Manager::_changeOneCrossLaneConstraint(bool print){
    //TODO
}
