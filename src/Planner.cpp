#include "Planner.h"

State DifferentialRobot::nextState(State const& s, Action a) const
{
    float dt = 0.1f;
    switch(a)
    {
        case STRAIGHT:
        {
            return State(s.x + _velocity*cos(s.yaw)*dt, 
                          s.y + _velocity*sin(s.yaw)*dt,
                          s.yaw
                        );
        }

        case ROTATE:
        {
            return State(s.x, s.y, normalize(s.yaw + (2*_velocity*dt)/(2*_radius)));
        }
    }

    return State(s.x + _velocity*cos(s.yaw)*dt, 
                  s.y + _velocity*sin(s.yaw)*dt,
                  s.yaw
                );
}


vector<State> Planner::findPath(State const& start, State const& goal)
{
    _goal = goal;
    shared_ptr<Node> rootParent(nullptr);
    shared_ptr<Node> startNode = make_shared<Node>(toIndex(start), start, 0.0f, heuristic(start), rootParent);

    _pq.push(startNode);

    while(_pq.size() > 0)
    {
        shared_ptr<Node> top = _pq.top(); _pq.pop();

        if(isGoal(top->state))
        {
            vector<State> result;
            while(top->parent.get() != nullptr)
            {
                result.push_back(top->state);
                top = top->parent;
            }
            reverse(result.begin(), result.end());
            return result;
        }
        
        for(int action=STRAIGHT; action<MAXACTION; action++)
        {
            State ns = _robot.nextState(top->state, (Action) action);     
            shared_ptr<Node> neighbor = make_shared<Node>(toIndex(ns), ns, top->cost + top->state.distanceTo(ns), heuristic(ns), top);
            _pq.push(neighbor);
        }
    
    }           

    return vector<State>(); 
}

long Planner::toIndex(State const& s) const
{
    long idx = (long)std::floor( (normalize(s.yaw, 0.0f) / TwoPI) * _yawIntervals) % _yawIntervals;

    long n = (long) std::floor( ( (s.y - _ymin) / (_ymax - _ymin) ) * _yIntervals);
    idx = idx * _yIntervals + n;

    n = (long) std::floor( ( (s.x - _xmin) / (_xmax - _xmin) ) * _xIntervals);
    idx = idx * _xIntervals + n;

    return idx;
}

bool Planner::isGoal(State const& s) const
{
    return  (      (std::fabs(s.x - _goal.x) < _goalXTol)
                && (std::fabs(s.y - _goal.y) < _goalYTol)
                && (std::fabs(s.yaw - _goal.yaw) < _goalYawTol)
            );
}

float Planner::heuristic(State const& s) const
{
    return s.distanceTo(_goal);
}

