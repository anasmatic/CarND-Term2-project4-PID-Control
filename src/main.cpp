#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double kp = 0.12, ki = 0, kd = 3;//manually tuned parameters, will be used on initial state
double p[] = { 0.12,0,3 };
double pd[] = { 1,0,15 };
double throttle = 0.4;
int count = 0;
double tolerance = 0.1;
int n = 5; //steps to publish next new error in twiddle
int i = 0;//iterate over p and/or pd param
void next_i() {
	i++;
	if (i == 3)i = 0;//out of index, reset to 0
	if (i == 1) i = 2;//skip ki
}
bool isUsingTwiddle = true;//default
bool flag_to_1st_step = true;//flag to twiddle first step
bool flag_to_else = false;//flag to 2nd stage in twiddle second step
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_last_of("]");
	if (found_null != std::string::npos) {
		return "";
	}
	else if (b1 != std::string::npos && b2 != std::string::npos) {
		return s.substr(b1, b2 - b1 + 1);
	}
	return "";
}

int main(int argc, char *argv[])
{
	uWS::Hub h;

	PID pid;
	// TODO: Initialize the pid variable.
	//by experment, 
	//the more Kp , the more car swings after turn, wee, wee, wee, wee !
	//				but the least, the less the car will turn on prober time
	//for testing
	double err = 0;
	double best_err = 1000;
	if (argc > 1) {
		for (int i = 1; i < argc; i++) {
		//int i = 1;
			if (strcmp(argv[i],"t") == 0) {
				
				std::cout << "set t" << std::endl;
				i++;tolerance = atof(argv[i]);
			}
			if (strcmp(argv[i],"n") == 0) {
				isUsingTwiddle = true;
				std::cout << "set n" << std::endl;
				i++;n = atoi(argv[i]);
			}
			if (strcmp(argv[i],"p") == 0) {
				std::cout << "set p" << std::endl;
				i++;p[0] = atof(argv[i]);
				i++; p[1] = atof(argv[i]);
				i++; p[2] = atof(argv[i]);
			}
			if (strcmp(argv[i],"d") == 0) {
				std::cout << "set d" << std::endl;
				i++; pd[0] = atof(argv[i]);
				i++; pd[1] = atof(argv[i]);
				i++; pd[2] = atof(argv[i]);
			}
			if (strcmp(argv[i],"x") == 0) {
				i++; kp = atof(argv[1]);
				i++; ki = atof(argv[2]);
				i++; kd = atof(argv[3]);
			}
			if (strcmp(argv[i], "s") == 0) {

				std::cout << "set s" << std::endl;
				i++; throttle = atof(argv[i]);
			}
		}
	}
	std::cout << "n:" << n << std::endl;
	std::cout << "speed:" << throttle << std::endl;
	std::cout << "tolerance:" << tolerance << std::endl;
	std::cout << "p:" << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
	std::cout << "pd:" << pd[0] << ", " << pd[1] << ", " << pd[2] << std::endl;
	std::cout << "kp:" << kp << ", ki:" << ki<< ", kd:" << kd << std::endl;
	//pid.Init(0.15, 0, -3.0);
	pid.Init(kp, ki, kd);

	h.onMessage([&pid, &err, &best_err](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{
			auto s = hasData(std::string(data).substr(0, length));
			if (s != "") {
				auto j = json::parse(s);
				std::string event = j[0].get<std::string>();
				if (event == "telemetry") {
					// j[1] is the data JSON object
					double cte = std::stod(j[1]["cte"].get<std::string>());
					double speed = std::stod(j[1]["speed"].get<std::string>());
					double angle = std::stod(j[1]["steering_angle"].get<std::string>());
					double steer_value;
					
					pid.UpdateError(cte);
					steer_value = pid.TotalError();
					if (isUsingTwiddle) {
						double sum_pd = pd[0] + pd[1] + pd[2];
						if (sum_pd > tolerance)//if true we keep doing the twiddle ~(1)
						{
							count++;
							err += (cte*cte) / count; //keep error total error updated					
							//initial step, upon car engien start
							if (count == n) {//do this step if only #n step has passed
								best_err = err;//save it as initial best_error
								err = 1000;//reset new_error
								std::cout << " 1 - best_err:" << best_err << " vs err:" << err << std::endl;
							}
							//now we twiddle until sum_pd > tolerance
							else if (count > n && count % n == 0) {//do this step if only #n step has passed
								//******** this is the first step of twiddle *********
								//it has flag_to_1st_step , 
								//this flag is true by default, and true after every loop is done
								if (flag_to_1st_step) {
									p[i] += pd[i];//change params
									flag_to_1st_step = false;//set false to skip if param index didn't change
								}//else loop anothe #n and check again
								else {
									//******** this is the second step of twiddle *********
									//this step starts with check and pd edit, then after #n loops in contine to else lines
									if (!flag_to_else && err < best_err) {//
										best_err = err;
										pd[i] *= 1.1;
										next_i();//now lets check next param. and "continue" next loop iteration
										flag_to_1st_step = true;
									}
									else {//this is 2nd stage of twiddle 2nd step,
										//stage 2 starts with a reset to p , then loop #n steps
										if (flag_to_else == false) {
											p[i] -= 2 * pd[i];
											flag_to_else = true;//means next time skip here ang go directly to elseif
											//then loop another #n, then check again, but remember to start from elseif next time
										}
										// after #n loops check again
										else if (flag_to_else) {
											if (err < best_err) {
												best_err = err;
												pd[i] *= 1.1;
											}
											else {
												p[i] += pd[i];
												pd[i] *= 0.9;
											}
											flag_to_else = false;//rest the flag, we are done with else
											next_i();//now lets check next param
											flag_to_1st_step = true;
										}

									}
								}
//								std::cout << " 2 - best_err:" << best_err << " vs err:" << err << std::endl;
//								std::cout << "			new " << p[0] << "," << p[1] << "," << p[2] << std::endl;
//								std::cout << "______________ sum dp" << sum_pd << std::endl;
								count = n;//reset count , not to  0 , but to #n 
								err = 0;//reset new_error
							}//end else if
							pid.Init(p[0], p[1], p[2]);//then update the kp ki kd params
						}//else just update stearing using the same Kp Ki Kd, we should be stable now with good error
						else {
							isUsingTwiddle = false;
							std::cout << " Twiddle Stopped" << p[0] << "," << p[1] << "," << p[2] << std::endl;
						}
					}

					//std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

					json msgJson;
					msgJson["steering_angle"] = steer_value;
					msgJson["throttle"] = throttle;//0.3;
					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					//std::cout << msg << std::endl;
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1)
		{
			res->end(s.data(), s.length());
		}
		else
		{
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
