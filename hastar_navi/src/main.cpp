#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include "hybrid_astar.h"

#include <chrono>
typedef std::chrono::high_resolution_clock Clock;

using namespace std;

int X = 1;
int _ = 0;

double SPEED  = 1.45;
double LENGTH = 0.5;

vector< vector<int> > MAZE = {
    {_,X,X,_,_,_,_,_,_,_,X,X,_,_,_,_,},
    {_,X,X,_,_,_,_,_,_,X,X,_,_,_,_,_,},
    {_,X,X,_,_,_,_,_,X,X,_,_,_,_,_,_,},
    {_,X,X,_,_,_,_,X,X,_,_,_,X,X,X,_,},
    {_,X,X,_,_,_,X,X,_,_,_,X,X,X,_,_,},
    {_,X,X,_,_,X,X,_,_,_,X,X,X,_,_,_,},
    {_,X,X,_,X,X,_,_,_,X,X,X,_,_,_,_,},
    {_,X,X,X,X,_,_,_,X,X,X,_,_,_,_,_,},
    {_,X,X,X,_,_,_,X,X,X,_,_,_,_,_,_,},
    {_,X,X,_,_,_,X,X,X,_,_,X,X,X,X,X,},
    {_,X,_,_,_,X,X,X,_,_,X,X,X,X,X,X,},
    {_,_,_,_,X,X,X,_,_,X,X,X,X,X,X,X,},
    {_,_,_,X,X,X,_,_,X,X,X,X,X,X,X,X,},
    {_,_,X,X,X,_,_,X,X,X,X,X,X,X,X,X,},
    {_,X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,},
    {X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,},
};


vector< vector<int> > GRID = MAZE;

vector<double> START = {0.0,0.0,0.0};
vector<int>    GOAL  = {(int)GRID.size()-1, (int)GRID[0].size()-1};


// add some heuristic methods to decrease expand numbers
vector<string> heuristic_methods = {"Manhattan",
                                    "Chebyshev",
                                    "Octile",
                                    "Euclidean",
                                    "Octile_breaktie"};

int main() {




  for (auto &heuristic : heuristic_methods){

    HAS has      = HAS();

    auto start = Clock::now();
    cout<< "================== ================== =================="<< endl;
    cout<< "Heuristic methods: " << heuristic << endl;
    HAS::grid_path get_path = has.search_heap(GRID, START, GOAL, heuristic);
    auto end = Clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    cout<< " time processed: " << dur << " milliseconds" << endl;

    // Show path
    vector<HAS::Node3D> show_path     = has.retrace_path(get_path.came_from, START, get_path.final);

    vector<HAS::Node3D> smoothed_path = has.smooth_path(show_path, 0.5, 0.1, 0.00001);
    
    // write to output file

    std::ofstream output_file("/home/mouse152n-04u/TEST/Hybrid_Astar/"+ std::string(heuristic) + "_method.txt");

    for(auto &step: show_path) {

        if (output_file) {
          //std::clog << "*** [INFO] Writing found path to file " << "\n" ;
          output_file << step.x << " " << step.y << " " << step.theta << "\n";
        }
        else std::clog << "*** [BUG] Unable to open file: " << "'\n" ;

    }

    std::ofstream output_file_smoothed("/home/mouse152n-04u/TEST/Hybrid_Astar/"+ std::string(heuristic) + "_method_smoothed.txt");

    for(auto &smooth: smoothed_path) {

        if (output_file_smoothed) {
          /*
          cout << "##### step " << smooth.g << " #####" << endl;
          cout << "x " << smooth.x << endl;
          cout << "y " << smooth.y << endl;
          cout << "theta " << smooth.theta << endl;
          */
          std::clog << "*** [INFO] Writing smoothed path to file " << "\n" ;
          output_file_smoothed << smooth.x << " " << smooth.y << " " << smooth.theta << "\n";
        }
        else std::clog << "*** [BUG] Unable to open file: " << "'\n" ;

    }


  }

  return 0;
}
