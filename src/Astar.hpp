#include <iostream>
#include <memory>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
using namespace std;

const float PI    = 3.14f;
const float TwoPI = 2*PI;

class State{
public:
    float x;
    float y;
    float yaw; //rad
    State(float const x=0.0f, float const y=0.0f, float const yaw=0.0f):x(x), y(y), yaw(yaw) {}
    float distanceTo (State const& that){
        return  (x - that.x)*(x - that.x) + (y-that.y)*(y-that.y);
    } 

    friend ostream& operator<< (std::ostream& os, State const & s){
        os << "(" << s.x << "," << s.y << "," << s.yaw << ",)" << endl;

        return os;
    }
};

enum Action{
    STRAIGHT,
    ROTATE,
    MAXACTION
};

class Node{
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

struct Cmp{
    bool operator () (shared_ptr<Node> const& n1, shared_ptr<Node> const& n2){
        return n1->total > n2->total;
    } 
};

class Astar{
public:
    
    //Constructor
    Astar(State const& goal) : _goal(goal), _pq() {}
    
    //Path finding function
    vector<State> findPath(State const& start)
    {
        shared_ptr<Node> rootParent(nullptr);
        shared_ptr<Node> startNode = make_shared<Node>(toIndex(start), start, 0.0f, heuristic(start), rootParent);

        _pq.push(startNode);

        while(_pq.size() > 0){
            shared_ptr<Node> top = _pq.top(); _pq.pop();

            //cout<<top->state;
            //std::cin.get();

            if(isGoal(top->state)){
                vector<State> result;
                while(top->parent.get() != nullptr){
                    result.push_back(top->state);
                    //cout<<top->state;
                    //cin.get();
                    top = top->parent;
                }
                reverse(result.begin(), result.end());
                return result;
            }
            
            for(int action=STRAIGHT; action<MAXACTION; action++){
                State ns = nextState(top->state, (Action) action);     
                //cout<<"["<<ns; 
                shared_ptr<Node> neighbor = make_shared<Node>(toIndex(ns), ns, top->cost + top->state.distanceTo(ns), heuristic(ns), top);
                _pq.push(neighbor);
            }
        
        }           

        return vector<State>(); 
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

    bool isGoal(State s){
        return  (      (std::fabs(s.x - _goal.x) < goalXTol)
                    && (std::fabs(s.y - _goal.y) < goalYTol)
                    && (std::fabs(s.yaw - _goal.yaw) < goalYawTol)
                );
    }

    float heuristic(State s){
        return s.distanceTo(_goal);
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
                return State(s.x, s.y, normalize( s.yaw + 2*v*dt/len));
                break;
            }
        }
    }

    std::priority_queue<shared_ptr<Node>, vector<shared_ptr<Node>>, Cmp> _pq; 
    unsigned int _xIntervals   =  100;    //0.2m bin size
    unsigned int _yIntervals   =  100;    //0.2m bin size
    unsigned int _yawIntervals =  120;     // 3.0 deg bin size
    float _xmin                = -100.0f;
    float _ymin                = -100.0f;
    float _xmax                =  100.0f;
    float _ymax                =  100.0f;
    float goalXTol             =  2.0f;
    float goalYTol             =  2.0f;
    float goalYawTol           =  3.0f * PI/180.0f;   //3.0 deg

    State _goal;

};

