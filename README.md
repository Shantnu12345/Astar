# CPPND: Capstone Hello World Repo
For the capstone, I chose to implement an Astar path planning algorithm that finds path for a non-holonomic differential drive robot from a start position to an end position.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./HelloWorld`.
5. The result will get saved in "path.csv"

## Code structure
Code has a src/Astar.hpp file with all the functions and classes relevant to the Astar path finding algorithm. The main() function is in src/main.cpp

## An explanation of how my submission satisfies the necessary rubric

### REQUIRED
1. A README with instructions is included with the project
2. The README indicates which project is chosen.
3. The README includes information about each rubric point addressed.
4. The submission compiles and runs.

### RUBRICS PASSED:
1. The project makes use of references in function declarations: See src/Astar.hpp::58
2. The project uses smart pointers instead of raw pointers: See src/Astar.hpp::42
3. The project uses Object Oriented Programming techniques: See src/Astar.hpp::54
4. Classes use appropriate access specifiers for class members: see src/Astar.hpp::36
5. Class constructors utilize member initialization lists: see src/Astar.hpp::44
6. The project reads data from a file and process the data, or the program writes data to a file.

There are several other RUBRICS satisfied, however, I haven't written them here.








 
