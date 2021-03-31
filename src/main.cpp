#include "Planner.h"
#include <fstream>

void to_csv(vector<State> const& path){
    std::ofstream ofs("Path.csv");
    ofs << "x,y" << std::endl;
    for(auto const& point:path){
        ofs << point.x << "," << point.y << endl;
    }
}

int main() {
    State start(0.0f, 0.0f, 0.0f);
    State goal(10.0f, 0.0f, PI/2);
    DifferentialRobot robot(1.0f, 0.25f); 
    Planner planner(robot);
    vector<State> path = planner.findPath(start, goal);
    std::cout << "Hurray!! Found path with " << path.size() << " waypoints" << endl ;
    to_csv(path);
    return 0;
}