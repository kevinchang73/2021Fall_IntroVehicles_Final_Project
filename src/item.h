#ifndef ITEM_H
#define ITEM_H

#include <vector>
#include <map>
#include <set>
#include <limits>
using namespace std;

class Edge;
class Node;
enum Edge_type{
    SAME_LANE_PATH = 0,
    CROSS_LANE_PATH = 1,
    SAME_LANE_CONSTRAINT = 2,
    CROSS_LANE_CONSTRAINT = 3,
    PSUEDO = 4
};
enum Trajectory_type{
    ON_TO_ON = 0,
    ON_TO_OFF = 1,
    OFF_TO_ON = 2,
    OFF_TO_OFF = 3
};

class Vehicle
{
//friend class Manager;
public:
    Vehicle() {}
    Vehicle(int id, int lane, bool exit, double eat):
        _id(id), _incomingLane(lane), _toExit(exit), _earliestAT(eat) {}
    ~Vehicle() {}

    void print() {
        cout << "Vehicle " << _id << ":\n";
        cout << "Coming lane = " << _incomingLane << "\n";
        cout << "To exit = " << _toExit << "\n";
        cout << "Earliest arrival time = " << _earliestAT << "\n";
    }
//private:
    //given
    int _id;
    int _incomingLane;
    bool _toExit;
    double _earliestAT;
    Trajectory_type _type;
    //graph
    vector< vector<Node*> > _nodes; //from incoming lane to exit lane(s)
    //solution
    vector<int> _changePoints;
};

class Node
{
//friend class Manager;
public:
    Node() {}
    Node(int l, int p, Vehicle* v):
        _lane(l), _point(p), _vehicle(v) {}
    ~Node() {}
    void print(){
        if(_vehicle) cout << _vehicle->_id << " " << _lane << " " << _point;
        else cout << "-1 " << _lane << " " << _point;
    }
//private:
    // Node info
    int _lane;
    int _point;
    Vehicle* _vehicle;
    // Edges
    Edge* _outSameLaneEdge = 0;
    Edge* _outCrossLaneEdge = 0;
    Edge* _inSameLaneEdge = 0;
    Edge* _inCrossLaneEdge = 0;
    Edge* _inPsuedoEdge = 0;
    vector<Edge*> _outSameLaneConsEdges;
    vector<Edge*> _inSameLaneConsEdges;
    vector<Edge*> _outCrossLaneConsEdges;
    vector<Edge*> _inCrossLaneConsEdges;
    // Current solution
    int _priority; //the priority on the lane (the lower the better)
    bool _enable = 0; //is extracted in the current solution
    int _extractedIdx = -1; //node index in extracted graph
    double _arrivalTime = -numeric_limits<double>::max(); //(earliest) arrival time under the current solution
    // Best solution
    double _bestArrivalTime = -1;
};

class Edge
{
//friend class Manager;
public:
    Edge() {}
    Edge(Node* u, Node* v, Edge_type type, double weight):
        _type(type), _u(u), _v(v), _weight(weight) {}
    ~Edge() {}
    void reverse() {
        Node* temp = _u;
        _u = _v;
        _v = temp;
    }
    bool enable() {
        if(_type == Edge_type::CROSS_LANE_CONSTRAINT){
            Node* inn1 = _u->_inCrossLaneEdge->_u;
            Node* out1 = _v->_inCrossLaneEdge->_u;
            return _u->_enable && _v->_enable && inn1->_enable && out1->_enable;
        }
        else{
            return _u->_enable && _v->_enable;
        }
    }
    void print() {
        cout << "("; _u->print(); cout << ") ";
        cout << "("; _v->print(); cout << ") ";
        cout << "weight = " << _weight << " ";
    }
//private:
    Edge_type _type;
    Node* _u;
    Node* _v;
    double _weight;
};

class Point
{
//friend class Manager;
public:
    Point() {}
    Point(int lane, int k):
        _lane(lane), _k(k) {}
    ~Point() {}
//private:
    int _lane;
    int _k; //discrete point id
    map<int, Node*> _vehicleID2Node;
};

class Lane
{
public:
    Lane() {}
    Lane(int idx): _idx(idx) {}
    ~Lane() {}
//private:
    int _idx;
    vector<Point*> _points;
    vector<int>    _order; //priority order of each vehicle
    set<int>       _passVehicles; //IDs of possible passing vehicles
};

#endif //ITEM_H
