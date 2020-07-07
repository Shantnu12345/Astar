#include <iostream>
#include <memory>
#include <vector>
#include <queue>
#include <cmath>
using namespace std;

const bool PI    = 3.14f;
const bool TwoPI = 2*PI;

class State{
public:
    float x;
    float y;
    float yaw; //rad
    State(float const x, float const y, float const yaw):x(x), y(y), yaw(yaw) {}
};

enum Action{
    STRAIGHT,
    ROTATE
};

class Node{
public:
    int   idx;
    State state;
    float cost;
    float heuristic;
    float total;
    unique_ptr<Node> parent;

    Node(int idx, State const& s, float cost, float heuristic, float total, unique_ptr<Node> parent) :
        idx(idx), state(s), cost(cost), heuristic(heuristic), total(total), parent(move(parent)){}
};

struct Cmp{
    bool operator () (Node const& n1, Node const& n2){
        return n1.total < n2.total;
    } 
};

class Astar{
public:
    Astar(State const& goal) : _goal(goal) {}
    vector<State> findPath(State const& start){
        vector<State> result(0, State(0.0f, 0.0f, 0.0f));
        
        Node startNode()
        
        
        
        
        
        
        
        
        return result; 
    }

protected:
    long toIndex(State s){
        long idx = (long)std::floor( (normalize(s.yaw, 0.0f) / TwoPI) * _yawIntervals) % _yawIntervals;

        long n = (long) std::floor( ( (s.y - _ymin) / (_ymax - _ymin) ) * _yIntervals);
        idx = idx * _yIntervals + n;
        // n = ba::clamp(n, 0, YIntervals - 1);

        n = (long) std::floor( ( (s.x - _xmin) / (_xmax - _xmin) ) * _xIntervals);
        idx = idx * _xIntervals + n;

        return idx;
    }

    float heuristic(State s){
        return std::sqrt( (s.x - _goal.x)*(s.x - _goal.x) + (s.y - _goal.y)*(s.y - _goal.y) );
    }

    float normalize(float angle, float ref = -PI){
        float temp = std::fmod(angle - ref, TwoPI);
        return temp >= 0.0f ? temp + ref: temp + ref + TwoPI;
    }

    State nextState(State const& s, Action a){
        float dt = 0.1f;
        float v = 1.0f;
        float len = 0.25f;
        switch(a){
            case STRAIGHT:
            {
                return State(s.x + v*cos(s.yaw)*dt, 
                             s.y + v*sin(s.yaw)*dt,
                             s.yaw
                            );
                break;
            }

            case ROTATE:
            {
                return State(s.x, s.y, s.yaw + 2*v*dt/len);
            }
        }
    }

    std::priority_queue<Node, vector<Node>, Cmp> _pq; 
    unsigned int _xIntervals   =  100;    //0.2m tolerance
    unsigned int _yIntervals   =  100;    //0.2m tolerance
    unsigned int _yawIntervals =  120;     // 3.0 deg tolerance
    float _xmin                = -100.0f;
    float _ymin                = -100.0f;
    float _xmax                =  100.0f;
    float _ymax                =  100.0f;
    State _goal;

};

