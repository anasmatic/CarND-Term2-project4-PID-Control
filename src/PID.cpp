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
	//used to update new kp ki kd parameters
	this->Kp = Kp;	this->Ki = Ki;	this->Kd = Kd;
}

void PID::UpdateError(double cte) {
	if (is_initialized == false) {
		p_error = cte;//don't init p_error to 0
		is_initialized = true;
	}
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
}

double PID::TotalError() {
	double total_error = (-Kp * p_error) - (Kd * d_error) - (Ki * i_error);
	return total_error;
}

