#ifndef ITEM_H
#define ITEM_H

#include <vector>
#include <map>
using namespace std;

class Edge;
enum Edge_type{
    SAME_LANE_PATH = 0,
    CROSS_LANE_PATH = 1,
    SAME_LANE_CONSTRAINT = 2,
    CROSS_LANE_CONSTRAINT = 3
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
    int _id;
    int _incomingLane;
    bool _toExit;
    double _earliestAT;
    int _exitLane = 0;
};

class Node
{
//friend class Manager;
public:
    Node() {}
    Node(int l, int p, Vehicle* v):
        _lane(l), _point(p), _vehicle(v), _outSameLaneEdge(0), _outCrossLaneEdge(0), _inSameLaneEdge(0), _inCrossLaneEdge(0) {}
    ~Node() {}
//private:
    int _lane;
    int _point;
    Vehicle* _vehicle;
    Edge* _outSameLaneEdge;
    Edge* _outCrossLaneEdge;
    Edge* _inSameLaneEdge;
    Edge* _inCrossLaneEdge;
    vector<Edge*> _outConsEdges;
    vector<Edge*> _inConsEdges;
};

class Edge
{
//friend class Manager;
public:
    Edge() {}
    Edge(Node* u, Node* v, Edge_type type):
        _type(type), _u(u), _v(v) {}
    ~Edge() {}
    void reverse() {
        Node* temp = _u;
        _u = _v;
        _v = temp;
    }
//private:
    Edge_type _type;
    Node* _u;
    Node* _v;
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

#endif //ITEM_H
