#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

int main() {

	//declare variables
	double m, k, x, v, t_max, dt, t, a;
	vector<double> x_list, v_list, t_list;

	//initial condition
	m = 1;
	k = 1;
	x = 0;
	v = 1;
	t_max = 50;
	dt = 0.1;
	
	//when t=0
	x_list.push_back(x);
	v_list.push_back(v);
	x = x + dt * v;
	x_list.push_back(x);

	//Verlet integration
	for (t = 1; t <= t_max; t = t + dt) {
		t_list.push_back(t);
		x_list.push_back(x);
		v_list.push_back(v);

		//calc new position and velocity
		a = -k * x / m;
		x = 2 * x_list[-1] - x_list[-2] + pow(dt, 2) * a;
		v = (x - x_list[-1]) / dt;
	}

	//write trajectory to file
	ofstream fout;
	fout.open("verlet_traj.txt");
	if (fout) {
		for (int i = 0; i < t_list.size(); i = i + 1) {
			fout << t_list[i] << ' ' << x_list[i] << ' ' << v_list[i] << endl;
		}
	}
	else { // file did not open successfully
		cout << "Could not open trajectory file for writing" << endl;
	}
	}