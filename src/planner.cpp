#include "planner.h"
#include <vector>
#include <math.h>

using namespace std;

vector < vector <double> > get_points(double x, double y, double r, int n){
	vector< vector< double> > points;
	for (int i = 0; i < n; ++i){
		double xt, yt;
		xt = x + r*cos(2*M_PI/n*i);
		yt = y - r*sin(2*M_PI/n*i);
		vector< double > xy;
		xy.push_back(xt);
		xy.push_back(yt);
		points.push_back(xy);
	}
	return points;
}

