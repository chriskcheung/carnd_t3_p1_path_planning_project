#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
//#include "spline.h"
//#include "helper.h"
#include "path_planner.h"

using namespace std;

// for convenience
using json = nlohmann::json;

pathPlanning pp;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  int lane = 1; // left|middle|right : 0|1|2
  double ref_v = 49.5; //mph
  int iter = 0;
  const int trajectary_size = 50;
  
  h.onMessage([&lane,&ref_v,&iter,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
	iter++;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];  // radian
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	vector<double> previous_path_x = j[1]["previous_path_x"];
          	vector<double> previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

			int previous_path_size = previous_path_x.size();
			int prv_path_size = previous_path_x.size();

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
			//example of going straight
			/**
			double dist_inc = 0.5;
			for(int i = 0; i < 50; i++)
			{
				  next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
				  next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
			}			
			**/

			//example of doing donut
			/**
			double pos_x;
			double pos_y;
			double angle;
			int path_size = previous_path_x.size();
			for(int i = 0; i < path_size; i++)
			{
			  next_x_vals.push_back(previous_path_x[i]);
			  next_y_vals.push_back(previous_path_y[i]);
			}
			if(path_size == 0)
			{
			  pos_x = car_x;
			  pos_y = car_y;
			  angle = deg2rad(car_yaw);
			}
			else
			{
			  pos_x = previous_path_x[path_size-1];
			  pos_y = previous_path_y[path_size-1];
			  double pos_x2 = previous_path_x[path_size-2];
			  double pos_y2 = previous_path_y[path_size-2];
			  angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
			}
			double dist_inc = 0.5;
			for(int i = 0; i < 50-path_size; i++)
			{    
			  next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
			  next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
			  pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
			  pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
			}			
			**/

			//example of stay in lane
			/**
			double dist_inc = 0.4;
			for(int i = 0; i < trajectary_size; i++)
			{
				// Predict next s and d
				double next_s = car_s+(i+1)*dist_inc;
				double next_d = 2;  // within 2 <= 6 <= 10 for left|middle|right lane. potentially change lane by using cost function to determine
				vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
				next_x_vals.push_back(xy[0]);
				next_y_vals.push_back(xy[1]);
				//std::cout << "new next_x_vals[" << i << "]=" << next_x_vals[i] << endl;
				//std::cout << "new next_y_vals[" << i << "]=" << next_y_vals[i] << endl;
			}
			**/
			
			// experimental code (not working)
			/**
			double dist_inc = 0.4;
			vector<double> ptsx, ptsy;  // for temporary holding the points
			spline s;
			double ref_x = car_x;
			double ref_y = car_x;
			double ref_yaw = deg2rad(car_yaw);
			
			if (previous_path_size == 0) {
				std::cout << "[" << iter << "] =======================================" << endl;
				std::cout << "previous_path_x.size=" << previous_path_x.size() << endl;
				std::cout << "car_d=" << car_d << endl;
				
				for(int i = 0; i < trajectary_size; i++)
				{
					// Predict next s and d
					double next_s = car_s+(i+1)*dist_inc;
					double next_d = 2;  // potentially change lane by using cost function to determine
					vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					pts_x.push_back(xy[0]);
					pts_y.push_back(xy[1]);
					//std::cout << "new next_x_vals[" << i << "]=" << next_x_vals[i] << endl;
					//std::cout << "new next_y_vals[" << i << "]=" << next_y_vals[i] << endl;
				}
			}
			else {
				std::cout << "[" << iter << "] =======================================" << endl;
				std::cout << "previous_path_x.size=" << previous_path_x.size() << endl;
				// copy the previous path x and y
				for(int i = 0; i < previous_path_size; i++)
				{
					next_x_vals.push_back(previous_path_x[i]);
					next_y_vals.push_back(previous_path_y[i]);
					//std::cout << "prev next_x_vals[" << i << "]=" << next_x_vals[i] << endl;
					//std::cout << "prev next_y_vals[" << i << "]=" << next_y_vals[i] << endl;
				}
				
				// Predict next s and d
				double previous_path_x0 = previous_path_x[previous_path_size-1];
				double previous_path_y0 = previous_path_y[previous_path_size-1];
				double previous_path_x1 = previous_path_x[previous_path_size-2];
				double previous_path_y1 = previous_path_y[previous_path_size-2];
				double previous_path_theta = atan2(fabs(previous_path_y1-previous_path_y0),fabs(previous_path_x1-previous_path_x0));
				vector<double> next_sd = getFrenet(previous_path_x0, previous_path_y0, 
				                          previous_path_theta, map_waypoints_x, map_waypoints_y);
				std::cout << "last previous_path_x0=" << previous_path_x0 << endl;
				std::cout << "last previous_path_y0=" << previous_path_y0 << endl;
				std::cout << "last end_path_s=" << end_path_s << endl;
				std::cout << "last end_path_d=" << end_path_d << endl;
				std::cout << "last previous_path_x1=" << previous_path_x1 << endl;
				std::cout << "last previous_path_y1=" << previous_path_y1 << endl;
				for(int i = previous_path_size; i<trajectary_size; i++) {
					double next_s = next_sd[0]+(i+1)*dist_inc;
					double next_d = next_sd[1];
					vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					std::cout << "next_s"<<i<<"=" << next_s << endl;
					std::cout << "next_d"<<i<<"=" << next_d << endl;
					std::cout << "next_x"<<i<<"=" << xy[0] << endl;
					std::cout << "next_y"<<i<<"=" << xy[1] << endl;
					next_x_vals.push_back(xy[0]);
					next_y_vals.push_back(xy[1]);
				}
			}
			**/
			
			// walk through code
			// first, build a spline with 2 current/current-1 (x,y) points, or the last 2 points from
			// the end of previous trajectory-1 (x,y) + 3 (x,y) points that are 30 meters apart
			/***
			vector<double> ptsx, ptsy;
			double ref_x = car_x;
			double ref_y = car_y;
			double ref_yaw = deg2rad(car_yaw);
			
			if (prv_path_size < 2){
				//std::cout << "[" << iter << "] =======================================" << endl;
				//std::cout << "prv_path_size<2 =" << prv_path_size << endl;
					// use two points that make the path tangent to the car
				double prv_car_x = car_x - cos(car_yaw);
				double prv_car_y = car_y - sin(car_yaw);
				
				ptsx.push_back(prv_car_x);
				ptsx.push_back(car_x);
				ptsy.push_back(prv_car_y);
				ptsy.push_back(car_y);
			}
			// use the previous path's end point as starting reference
			else {
				//std::cout << "[" << iter << "] =======================================" << endl;
				//std::cout << "prv_path_size>2 =" << prv_path_size << endl;
				ref_x = previous_path_x[prv_path_size-1];
				ref_y = previous_path_y[prv_path_size-1];
				
				double ref_x_prv = previous_path_x[prv_path_size-2];
				double ref_y_prv = previous_path_y[prv_path_size-2];
				ref_yaw = atan2(ref_y-ref_y_prv, ref_x-ref_x_prv);
				
				// use two points that make the path tangent to the previous path's end point
				ptsx.push_back(ref_x_prv);
				ptsx.push_back(ref_x);
				ptsy.push_back(ref_y_prv);
				ptsy.push_back(ref_y);
			}
			//std::cout << "1st - ptsx/y.size=" << ptsx.size() << endl;
			// in Frenet add evenly 30m spaced points ahead of the starting reference so the points are not only cover the even distance points
			vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			
			ptsx.push_back(next_wp0[0]);
			ptsx.push_back(next_wp1[0]);
			ptsx.push_back(next_wp2[0]);
			
			ptsy.push_back(next_wp0[1]);
			ptsy.push_back(next_wp1[1]);
			ptsy.push_back(next_wp2[1]);
			
			//std::cout << "2nd - ptsx/y.size=" << ptsx.size() << endl;
			for(int i=0; i<ptsx.size(); i++){
				//shift car reference angle to 0 degrees to align with local coordinates or in car's prespective
				double shift_x = ptsx[i]-ref_x;
				double shift_y = ptsy[i]-ref_y;
				
				ptsx[i] = (shift_x*cos(0-ref_yaw)) - (shift_y*sin(0-ref_yaw));
				ptsy[i] = (shift_x*sin(0-ref_yaw)) + (shift_y*cos(0-ref_yaw));
			}
			
			// create a spline for fitting
			tk::spline s;
			
			// set (x,y) points to spline
			s.set_points(ptsx, ptsy);
			
			// Since the spline fitting line starts from the last 2 points from the previous path points
			// as a continuation and the rest of the points are newly plotted/picked to predict the new path 
			// for the next 30 meters. With the spline fitting line ready, break the spline evenly to N points with the distance and time interval
			// base on the the speed that we want to travel
			
			// start with all of previous path points from last time
			for(int i=0; i<prv_path_size; i++) {
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
				//std::cout << "prv - next_x["<<i<<"=" << next_x_vals[i] << endl;
				//std::cout << "prv - next_y["<<i<<"=" << next_y_vals[i] << endl;
			}
			
			// calculate how to break up spline fitting points so that we travel at our desired reference velocity
			double target_x = 30.0;  // meter
			double target_y = s(target_x);
			double target_dist = sqrt((target_x*target_x)+(target_y*target_y));
			double x_add_on = 0;
			
			// fill up the rest of out path planner after filling it with previous points, here we will always output 50 points
			for(int i=1; i<=trajectary_size-prv_path_size; i++){
				double N = (target_dist/(0.02*ref_v/2.24));  // base on meter/sec
				double x_point = x_add_on+(target_x/N);
				double y_point = s(x_point);
				
				//std::cout << "N|x_point|y_point=" << N << x_point << y_point << endl
				
				x_add_on = x_point;
				
				double x_ref = x_point;
				double y_ref = y_point;
				
				// rotate back to normal after rotating it eariler (back to global coordinates)
				x_point = (x_ref*cos(ref_yaw)) - (y_ref*sin(ref_yaw));
				y_point = (x_ref*sin(ref_yaw)) + (y_ref*cos(ref_yaw));
				
				x_point += ref_x;
				y_point += ref_y;
				
				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);
				//std::cout << "new - next_x["<<prv_path_size-1+i<<"=" << next_x_vals[prv_path_size-1+i] << endl;
				//std::cout << "new - next_y["<<prv_path_size-1+i<<"=" << next_y_vals[prv_path_size-1+i] << endl;
			}
			**/
			
			// TODO:
			// 1) from current state, find all possible state
			// 2) from all possible state, find the cost of updating to possible states
			// 3) pick the lowest cost lane to proceed
			// 4) from the picked state, figure out the future path 
			
			// initialize all cost variables before processing
			pp.initPathPlanning(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
			cout << "[" << iter << "] initPathPlanning done" << endl;
			
			// process sensor data fron sensor fusion
			pp.processSensorData(car_x, car_y, car_s, car_d, car_yaw, car_speed, previous_path_x, previous_path_y, end_path_s, end_path_d, sensor_fusion);
			cout << "[" << iter << "] processSensorData done" << endl;
			
			// update states in FWM, determine next state and generate target_lane
			pp.updateFSM();
			cout << "[" << iter << "] updateFSM done" << endl;
			
			// generate a trajectory base on targeting lane determined by FSM
			pp.generateTrajectory(previous_path_x, previous_path_y);
			cout << "[" << iter << "] generateTrajectory done" << endl;
			cout << "=======================================" << endl;

			// END
			msgJson["next_x"] = pp.next_x;
          	msgJson["next_y"] = pp.next_y;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}