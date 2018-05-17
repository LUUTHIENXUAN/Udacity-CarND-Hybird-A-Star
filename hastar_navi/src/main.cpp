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


  HAS has      = HAS();
  HAS has_heap = HAS();

  for (auto &heuristic : heuristic_methods){


      auto start = Clock::now();
      cout<< "================== ================== =================="<< endl;
      cout<< "Heuristic methods: " << heuristic << endl;
      HAS::maze_path get_path = has.search_heap(GRID, START, GOAL, heuristic);
      auto end = Clock::now();
      auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
      cout<< " time processed: " << dur << " milliseconds" << endl;

      /*
      cout<< " ------------------ ------------------ ------------------"<< endl;
      start = Clock::now();
      HAS::maze_path get_path_heap = has_heap.search_heap(GRID, START, GOAL, heuristic);
      end = Clock::now();
      dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
      cout<< " time processed: " << dur << " milliseconds" << endl;
      */

      // Show path
      vector<HAS::maze_s> show_path = has.reconstruct_path(get_path.came_from, START, get_path.final);

      // write to output file

      std::ofstream output_file("/home/mouse152n-04u/TEST/Hybrid_Astar/"+ std::string(heuristic) + "_method.txt");

      for(int i = show_path.size()-1; i >= 0; i--) {

          HAS::maze_s step = show_path[i];
          cout << "##### step " << step.g << " #####" << endl;
          cout << "x " << step.x << endl;
          cout << "y " << step.y << endl;
          cout << "theta " << step.theta << endl;

          if (output_file) {

          std::clog << "*** [INFO] Writing to file " << "\n" ;
          output_file << step.x << " " << step.y << " " << step.theta << "\n";
          }
          else std::clog << "*** [BUG] Unable to open file: " << "'\n" ;

      }


  }

  return 0;
}
