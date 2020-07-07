#include <iostream>
#include "Astar.hpp"
#include <fstream>

void to_csv(vector<State> const& path){
    std::ofstream ofs("Path.csv");
    ofs << "x,y" << std::endl;
    for(auto const& point:path){
        ofs << point.x << "," << point.y << endl;
    }
}

int main() {
    Astar astarAlgo(State(0.0f, 0.0f, PI));
    vector<State> path = astarAlgo.findPath(State(10.0f, 0.0f, 0.0f));
    std::cout << "Hurray!! Found path with " << path.size() << " waypoints" << endl ;
    to_csv(path);
    return 0;
}