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
	int max_turnable   = 30; //degree
	int turning_res    =  5; //degree

	double SPEED = 1.45;
	double LENGTH = 0.5;

	struct Node3D {

		int g;	// iteration
		int f;
		double x;
		double y;
		double theta;

		struct Node3D *parent;
	};

	struct grid_path {

		vector< vector< vector<int> > > closed;
		vector< vector< vector<Node3D> > > came_from;
		Node3D final;

	};


	/**
  	* Constructor
  	*/
 	HAS();

	/**
 	* Destructor
 	*/
 	virtual ~HAS();

 	static bool compare_Node3D(const HAS::Node3D & lhs,
                             const HAS::Node3D & rhs);

  double heuristic(double x, double y,
                   vector<int> goal,
                   string heuristic_method );

  int turning_cost(HAS::Node3D current_state,
                   double next_angle);

 	int theta_to_stack_number(double theta);

  int idx(double float_num);

  vector<string> heuristic_methods;

  vector<Node3D> expand(Node3D state,
                        vector<int> goal,
                        string heuristic_method);

  grid_path search(vector< vector<int> > grid,
                   vector<double> start,
                   vector<int> goal,
                   string heuristic_method);

  grid_path search_heap(vector< vector<int> > grid,
                       vector<double> start,
                       vector<int> goal,
                       string heuristic_method);

  vector<Node3D> retrace_path(vector< vector< vector<Node3D> > > came_from,
                            vector<double> start,
                            HAS::Node3D final);

};

#endif
