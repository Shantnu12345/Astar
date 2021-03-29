#pragma once
#include "helper.h"

enum Action
{
    STRAIGHT,
    ROTATE,
    MAXACTION
};

class State
{
public:
    float x;
    float y;
    float yaw; //rad
    State(float const x=0.0f, float const y=0.0f, float const yaw=0.0f):x(x), y(y), yaw(yaw) {}
    float distanceTo (State const& that) const
    {
        return  (x - that.x)*(x - that.x) + (y-that.y)*(y-that.y);
    } 

    friend ostream& operator<< (std::ostream& os, State const & s)
    {
        os << "(" << s.x << "," << s.y << "," << s.yaw << ",)" << endl;
        return os;
    }
};

class DifferentialRobot
{
public:
    DifferentialRobot(int v, int r) : _velocity(v), _radius(r) {}
    State nextState(State const& s, Action a) const;
    
protected:
    float _velocity; //meter per sec
    float _radius;   //meters
};

class Node
{
public:
    int   idx;
    State state;
    float cost;
    float heur;
    float total;
    shared_ptr<Node> parent;

    Node(int idx, State const& s, float cost, float h, shared_ptr<Node> parent) :
        idx(idx), state(s), cost(cost), heur(h), total(cost + h), parent(parent){}
};

struct Cmp
{
    bool operator () (shared_ptr<Node> const& n1, shared_ptr<Node> const& n2){
        return n1->total > n2->total;
    } 
};

class Planner
{
public:
    
    Planner(DifferentialRobot const& robot) : _robot(robot), _pq() {}
    vector<State> findPath(State const& start, State const& goal);

protected:
    float heuristic(State const& s) const;
    bool isGoal(State const& s) const;
    long toIndex(State const& s) const;

    std::priority_queue<shared_ptr<Node>, vector<shared_ptr<Node>>, Cmp> _pq; 
    unsigned int _xIntervals   =  100;    //0.2m bin size
    unsigned int _yIntervals   =  100;    //0.2m bin size
    unsigned int _yawIntervals =  120;     // 3.0 deg bin size
    float _xmin                = -100.0f;
    float _ymin                = -100.0f;
    float _xmax                =  100.0f;
    float _ymax                =  100.0f;
    float _goalXTol             =  2.0f;
    float _goalYTol             =  2.0f;
    float _goalYawTol           =  3.0f * PI/180.0f;   //3.0 deg

    DifferentialRobot _robot;
    State _goal;
};

