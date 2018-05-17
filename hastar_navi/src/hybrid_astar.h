#ifndef HYBRID_ASTAR_H_
#define HYBRID_ASTAR_H_

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>

using namespace std;

class HAS {
public:

	int NUM_THETA_CELLS = 90;
	/*angle resolution when expanding to new postion
	 *between max turnable angles(left & right)
	 */
	int max_turnable   = 35; //degree
	int turning_res    =  5; //degree

	double SPEED = 1.45;
	double LENGTH = 0.5;

	struct maze_s {

		int g;	// iteration
		int f;
		double x;
		double y;
		double theta;
	};

	struct maze_path {

		vector< vector< vector<int> > > closed;
		vector< vector< vector<maze_s> > > came_from;
		maze_s final;

	};


	/**
  	* Constructor
  	*/
 	HAS();

	/**
 	* Destructor
 	*/
 	virtual ~HAS();

 	static bool compare_maze_s(const HAS::maze_s & lhs,
                             const HAS::maze_s & rhs);

  double heuristic(double x, double y,
                   vector<int> goal,
                   string heuristic_method );

  int turning_cost(HAS::maze_s current_state,
                   double next_angle);

 	int theta_to_stack_number(double theta);

  int idx(double float_num);

  vector<string> heuristic_methods;

  vector<maze_s> expand(maze_s state,
                        vector<int> goal,
                        string heuristic_method);

  maze_path search(vector< vector<int> > grid,
                   vector<double> start,
                   vector<int> goal,
                   string heuristic_method);

 maze_path search_heap(vector< vector<int> > grid,
                       vector<double> start,
                       vector<int> goal,
                       string heuristic_method);

vector<maze_s> retrace_path(vector< vector< vector<maze_s> > > came_from,
                            vector<double> start,
                            HAS::maze_s final);

};

#endif
