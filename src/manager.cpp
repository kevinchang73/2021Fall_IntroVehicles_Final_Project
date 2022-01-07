#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <cassert>
#include <vector>
#include <cmath>
#include <map>
#include <algorithm>
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
    _numRemianLanes = _numLanes - _numExitLanes;
    assert(_numDiscretePoints > max(_numExitLanes, _numRemianLanes));
    for(int i=0; i<_numVehicles; ++i){
        int id, lane;
        bool exit;
        double eat;
        input >> id >> lane >> exit >> eat;
        Vehicle* v = new Vehicle(id, lane, exit, eat);
        _vehicles.push_back(v);
    }
    if(PRINT){
        for(int i=0; i<_numVehicles; ++i){
            _vehicles[i]->print();
        }
    }
}

void Manager::plan() {
    _initialize();
}

void Manager::_initialize() {
    // discrete points
    for(int i=0; i<_numLanes; ++i){
        vector<Point*> lane;
        lane.reserve(_numDiscretePoints);
        for(int j=0; j<_numDiscretePoints; ++j){
            Point* point = new Point(i, j);
            lane.push_back(point);
        }
        _points.push_back(lane);
    }
    // graph nodes and path edges
    for(int i=0; i<_numVehicles; ++i){
        _initializeNodes(_vehicles[i]);
        _initializePaths(_vehicles[i]);
    }
}

void Manager::_initializeNodes(Vehicle* v, bool print){
    if(print) cout << "Vehicle " << v->_id << "\n";
    if(v->_toExit){
        if(v->_incomingLane < _numRemianLanes){ //change lane to exit
            int firstExitLane = _numRemianLanes;
            int crossLane = firstExitLane - v->_incomingLane + 1;
            for(int h=v->_incomingLane; h<=firstExitLane; ++h){
                for(int k=h-v->_incomingLane; k<_numDiscretePoints-crossLane+1+h-v->_incomingLane; ++k){
                    //int pointId = h - v->_incomingLane + k;
                    Node* node = new Node(h, k, v);
                    _points[h][k]->_vehicleID2Node[v->_id] = node;
                    if(print) cout << "(" << h << "," << k << ") ";
                }
            }
            for(int h=firstExitLane+1; h<_numLanes; ++h){
                for(int k=crossLane+h-(firstExitLane+1); k<_numDiscretePoints; ++k){
                    Node* node = new Node(h, k, v);
                    _points[h][k]->_vehicleID2Node[v->_id] = node;
                    if(print) cout << "(" << h << "," << k << ") ";
                }
            }
        }
        else{ //may or may not change lane to exit
            for(int h=v->_incomingLane; h<_numLanes; ++h){
                for(int k=h-v->_incomingLane; k<_numDiscretePoints; ++k){
                    Node* node = new Node(h, k, v);
                    _points[h][k]->_vehicleID2Node[v->_id] = node;
                    if(print) cout << "(" << h << "," << k << ") ";
                }
            }
        }
    }
    else{
        if(v->_incomingLane < _numRemianLanes){ //stay on the same lane to keep on the highway
            int h = v->_incomingLane;
            for(int k=0; k<_numDiscretePoints; ++k){
                Node* node = new Node(v->_incomingLane, k, v);
                _points[h][k]->_vehicleID2Node[v->_id] = node;
                if(print) cout << "(" << h << "," << k << ") ";
            }
        }
        else{ //change to inner lane to keep on the highway
            int firstRemainLane = _numRemianLanes - 1;
            int crossLane = v->_incomingLane - firstRemainLane + 1;
            for(int h=v->_incomingLane; h>=firstRemainLane; --h){
                for(int k=v->_incomingLane-h; k<_numDiscretePoints-crossLane+1+v->_incomingLane-h; ++k){
                    //int pointId = v->_incomingLane - h + k;
                    Node* node = new Node(h, k, v);
                    _points[h][k]->_vehicleID2Node[v->_id] = node;
                    if(print) cout << "(" << h << "," << k << ") ";
                }
            }
            for(int h=firstRemainLane-1; h>=0; --h){
                for(int k=_numDiscretePoints-crossLane+(firstRemainLane-1)-h; k<_numDiscretePoints; ++k){
                    Node* node = new Node(h, k, v);
                    _points[h][k]->_vehicleID2Node[v->_id] = node;
                    if(print) cout << "(" << h << "," << k << ") ";
                }
            }
        }
    }
    if(print) cout << "\n";
}

void Manager::_initializePaths(Vehicle* v, bool print){
    if(print) cout << "Vehicle " << v->_id << "\n";
    if(v->_toExit){
        for(int h=v->_incomingLane; h<_numLanes; ++h){
            for(int k=0; k<_numDiscretePoints-1; ++k){
                if(_points[h][k]->_vehicleID2Node.find(v->_id) != _points[h][k]->_vehicleID2Node.end()){
                    //cross-lane edges
                    if(h<_numLanes-1){
                        assert(_points[h+1][k+1]->_vehicleID2Node.find(v->_id) != _points[h+1][k+1]->_vehicleID2Node.end());
                        Node* node_u = _points[h][k]->_vehicleID2Node[v->_id];
                        Node* node_v = _points[h+1][k+1]->_vehicleID2Node[v->_id];
                        Edge* edge = new Edge(node_u, node_v, Edge_type::CROSS_LANE_PATH);
                        node_u->_outCrossLaneEdge = edge;
                        node_v->_inCrossLaneEdge = edge;
                        if(print) cout << "(" << node_u->_vehicle->_id << " " << node_u->_lane << " " << node_u->_point << "," << node_v->_vehicle->_id << " " << node_v->_lane << " " << node_v->_point<< ") ";
                    }
                    //same-lane edges
                    if(_points[h][k+1]->_vehicleID2Node.find(v->_id) != _points[h][k+1]->_vehicleID2Node.end()){
                        Node* node_u = _points[h][k]->_vehicleID2Node[v->_id];
                        Node* node_v = _points[h][k+1]->_vehicleID2Node[v->_id];
                        Edge* edge = new Edge(node_u, node_v, Edge_type::SAME_LANE_PATH);
                        node_u->_outSameLaneEdge = edge;
                        node_v->_inSameLaneEdge = edge;
                        if(print) cout << "(" << node_u->_vehicle->_id << " " << node_u->_lane << " " << node_u->_point << "," << node_v->_vehicle->_id << " " << node_v->_lane << " " << node_v->_point<< ") ";
                    }
                }
            }
        }
    }
    else{
        if(v->_incomingLane < _numRemianLanes){ //stay on the same lane to keep on the highway
            int h = v->_incomingLane;
            for(int k=0; k<_numDiscretePoints-1; ++k){
                assert(_points[h][k]->_vehicleID2Node.find(v->_id) != _points[h][k]->_vehicleID2Node.end());
                assert(_points[h][k+1]->_vehicleID2Node.find(v->_id) != _points[h][k+1]->_vehicleID2Node.end());
                Node* node_u = _points[h][k]->_vehicleID2Node[v->_id];
                Node* node_v = _points[h][k+1]->_vehicleID2Node[v->_id];
                Edge* edge = new Edge(node_u, node_v, Edge_type::SAME_LANE_PATH);
                node_u->_outSameLaneEdge = edge;
                node_v->_inSameLaneEdge = edge;
                if(print) cout << "(" << node_u->_vehicle->_id << " " << node_u->_lane << " " << node_u->_point << "," << node_v->_vehicle->_id << " " << node_v->_lane << " " << node_v->_point<< ") ";
            }
        }
        else{ //change to inner lane to keep on the highway
            for(int h=v->_incomingLane; h>=0; --h){
                for(int k=0; k<_numDiscretePoints-1; ++k){
                    if(_points[h][k]->_vehicleID2Node.find(v->_id) != _points[h][k]->_vehicleID2Node.end()){
                        //cross-lane edges
                        if(h>0){
                            assert(_points[h-1][k+1]->_vehicleID2Node.find(v->_id) != _points[h-1][k+1]->_vehicleID2Node.end());
                            Node* node_u = _points[h][k]->_vehicleID2Node[v->_id];
                            Node* node_v = _points[h-1][k+1]->_vehicleID2Node[v->_id];
                            Edge* edge = new Edge(node_u, node_v, Edge_type::CROSS_LANE_PATH);
                            node_u->_outCrossLaneEdge = edge;
                            node_v->_inCrossLaneEdge = edge;
                            if(print) cout << "(" << node_u->_vehicle->_id << " " << node_u->_lane << " " << node_u->_point << "," << node_v->_vehicle->_id << " " << node_v->_lane << " " << node_v->_point<< ") ";
                        }
                        //same-lane edges
                        if(_points[h][k+1]->_vehicleID2Node.find(v->_id) != _points[h][k+1]->_vehicleID2Node.end()){
                            Node* node_u = _points[h][k]->_vehicleID2Node[v->_id];
                            Node* node_v = _points[h][k+1]->_vehicleID2Node[v->_id];
                            Edge* edge = new Edge(node_u, node_v, Edge_type::SAME_LANE_PATH);
                            node_u->_outSameLaneEdge = edge;
                            node_v->_inSameLaneEdge = edge;
                            if(print) cout << "(" << node_u->_vehicle->_id << " " << node_u->_lane << " " << node_u->_point << "," << node_v->_vehicle->_id << " " << node_v->_lane << " " << node_v->_point<< ") ";
                        }
                    }
                }
            }
        }
    }
    if(print) cout << "\n";
}
