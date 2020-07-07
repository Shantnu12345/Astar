#include <iostream>
#include "Astar.hpp"

int main() {
    Astar astarAlgo(State(10.0f, 0.0f, PI));

    vector<State> path = astarAlgo.findPath(State(0.0f, 0.0f, 0.0f));
    cout << path.size();


    std::cout << "Hello World!" << "\n";
    return 0;
}