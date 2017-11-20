#include "PID.h"
#include <cmath>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	PID::Kp = Kp;	PID::Ki = Ki;	PID::Kd = Kd;
	p_error = i_error = d_error = 0;
//	p[0] = 0; p[1] = Ki; p[2] = Kd;
//	dp[0] = 1; dp[1] = 1; dp[2] = 1;
}

void PID::UpdateError(double cte) {
	if (is_initialized == false) {
		p_error = cte;//don't init p_error to 0
		is_initialized = true;
	}
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
	//cout << "		p_error:" << p_error<<endl;
	//cout << "		i_error:" << i_error << endl;
	//cout << "		d_error:" << d_error << endl;
}

double PID::TotalError() {
	double total_error = (-Kp * p_error) + (Kd * d_error) + (Ki * i_error);
	cout << "	total_error:" << total_error << endl;
	return total_error;
}

