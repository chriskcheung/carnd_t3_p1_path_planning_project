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
#include "spline.h"
#include "helper.h"
#include "path_planner.h"

using namespace std;

// return lane number base on lane position d
int pathPlanning::getLane(double d){
	return d/laneWidth_;
}


// reset all cost, closet distance variables and initialize all map variables
void pathPlanning::initPathPlanning(vector<double> &map_x, vector<double> &map_y, vector<double> &map_s, vector<double> &map_dx, vector<double> &map_dy){
	// initialize all cost and closet distance variables before processing
	for (int i=0; i<laneSize_; i++){
		cost_[i] = 0;
		closet_front_s_diff_[i] = min_safe_dist_;
		closet_back_s_diff_[i]  = min_safe_dist_;
		closet_front_id_[i] = 0;
		closet_back_id_[i]  = 0;
		closet_front_v_[i]  = 0.0;
		closet_back_v_[i]   = 0.0;
	}
	
	// initialize all map data
	map_x_  = map_x;
	map_y_  = map_y;
	map_s_  = map_s;
	map_dx_ = map_dx;
	map_dy_ = map_dy;
}

// process sensor data fron sensor fusion
void pathPlanning::processSensorData(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, vector<double> &previous_path_x, vector<double> &previous_path_y, double end_path_s, double end_path_d, vector<vector<double>> sensor_fusion){

	// ======================
	// load current car data
	// ======================
	car_x_ = car_x;
	car_y_ = car_y;
	car_s_ = car_s;
	car_d_ = car_d;
	car_yaw_ = car_yaw;
	car_speed_ = car_speed;
	car_v_mps_ = car_speed * MILE_PER_HOUR_2_METER_PER_SEC;
	car_lane_  = getLane(car_d_);
	

	// ======================
	// load previous path data and pick reference point
	// ======================
	prv_x_ = previous_path_x;
	prv_y_ = previous_path_y;
	prv_end_s_ = end_path_s;
	prv_end_d_ = end_path_d;
	prv_size_  = previous_path_x.size();
	
	// set reference point if last point of previous path exist otherwise current car's (x,y) position
	if(prv_size_ < 2){
		// none or not enough previous path points. use current car (x,y) position as starting point
		ref_x_ = car_x_;
		ref_y_ = car_y_;
		ref_s_ = car_s_;
		ref_d_ = car_d_;
		ref_yaw_ = deg2rad(car_yaw_);
		ref_v_ = car_v_mps_;
	}
	else {
		// pick the last 2 points of previous path trajectory and calculate their relative velocity
		// angle, and translate it to Frenet coordinate (car's coordinate)
		double prev_x1 = prv_x_[prv_size_-1];
		double prev_y1 = prv_y_[prv_size_-1];
		double prev_x2 = prv_x_[prv_size_-2];
		double prev_y2 = prv_y_[prv_size_-2];
		double prev_yaw = atan2(prev_y1-prev_y2, prev_x1-prev_x2);
		double prev_v_mps = distance(prev_x1, prev_y1, prev_x2, prev_y2)/dt_;
		vector<double> prev_sd = getFrenet(prev_x1, prev_y1, prev_yaw, map_x_, map_y_);
		
		// assign result to ref_* for calculating new path trajectory
		ref_x_ = prev_x1;
		ref_y_ = prev_x1;
		ref_s_ = prev_sd[0];
		ref_d_ = prev_sd[1];
		ref_yaw_ = prev_yaw;
		ref_v_ = prev_v_mps;
	}	
	ref_lane_ = getLane(ref_d_);
	

	// ======================
	// update cost functions base on surrounding vehicles
	// ======================
	// go through all sensor data of other vehicles to find the closet vehicle distance of each lane
	// format of sensor fusion vector
	// [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, 
	//  car's x velocity in m/s, car's y velocity in m/s, 
	//  car's s position in frenet coordinates, car's d position in frenet coordinates]
	
	double time_2_prev_path_end = dt_ * prv_size_;
	
	for (int i=0; i<sensor_fusion.size(); i++){
		double vehicle_id = sensor_fusion[i][0]; //vehicle unique ID
		double vehicle_x  = sensor_fusion[i][1]; //vehicle x position in map coordinates
		double vehicle_y  = sensor_fusion[i][2]; //vehicle y position in map coordinates
		double vehicle_vx = sensor_fusion[i][3]; //vehicle x velocity in m/s
		double vehicle_vy = sensor_fusion[i][4]; //vehicle y velocity in m/s
		double vehicle_s  = sensor_fusion[i][5]; //vehicle s position in frenet coordinates
		double vehicle_d  = sensor_fusion[i][6]; //vehicle d position in frenet coordinates
		
		// vehicle lane position
		double vehicle_lane = getLane(vehicle_d);
		
		// vehicle velocity
		double vehicle_v  = sqrt(vehicle_vx*vehicle_vx + vehicle_vy*vehicle_vy);
		
		// vehicle s distance with respect to end of subjected car's previous path trajectory
		double vehicle_s_at_prev_path_end = vehicle_s + vehicle_v*time_2_prev_path_end;
		
		// s diff between subjected car and vehicle from sensor fusion
		double s_diff = fabs(ref_s_ - vehicle_s_at_prev_path_end);
		
		// check if it is within visible range, otherwise skip
		if(s_diff > laneVisibleDist_){
			continue;
		}
		
		// check if vehicle is in the front
		if(ref_s_ > vehicle_s_at_prev_path_end){
			// check if vehicle is closer than previous recorded in its lane
			if(s_diff < closet_front_s_diff_[vehicle_lane]){
				closet_front_s_diff_[vehicle_lane] = s_diff;
				closet_front_id_[vehicle_lane] = vehicle_id;  // just in case if we need it for look up
				closet_front_v_[vehicle_lane] = vehicle_v;
			}
		}
		// vehicle is from the back 
		else {
			// check if vehicle is closer than previous recorded in its lane
			if(s_diff < closet_back_s_diff_[vehicle_lane]){
				closet_back_s_diff_[vehicle_lane] = s_diff;
				closet_back_id_[vehicle_lane] = vehicle_id;  // just in case if we need it for look up
				closet_back_v_[vehicle_lane] = vehicle_v;
			}
		}
	}
	
	
	// ======================
	// update cost functions
	// ======================
	// use the end of previous path info to determine the cost of changing lane
	switch(int(ref_lane_)){
		// left most lane
		case 0: 
				// not supporting to jump 2 lanes, so max cost for right most lane
				cost_[2] += 100;
				// cost of turn right base on any room on the right front and back, and the vehicle behind on the right was traveling faster than me
				cost_[1] += 100-closet_front_s_diff_[1] + (ref_v_ - closet_front_v_[1]) + 100-closet_back_s_diff_[1] - (ref_v_ - closet_back_v_[1]);
				// cost of staying in same lane base on the distance of vehicle in the front
				cost_[0] += 100-closet_front_s_diff_[0] + (ref_v_ - closet_front_v_[0]);
				break;

		// middle lane
		case 1: 
				// cost of turn right base on any room on the right front and back, and the vehicle behind was traveling faster than me
				cost_[2] += 100-closet_front_s_diff_[2] + (ref_v_ - closet_front_v_[2]) + 100-closet_back_s_diff_[2] - (ref_v_ - closet_back_v_[2]);
				// cost of staying in same lane base on the distance of vehicle in the front
				cost_[1] += 100-closet_front_s_diff_[1] + (ref_v_ - closet_front_v_[1]); 
				// cost of turn left base on any room on the left front and back, and the vehicle behind was traveling faster than me, 
				// right turn is preferrable, thus add 4 to left turn
				cost_[0] += 100-closet_front_s_diff_[0] + (ref_v_ - closet_front_v_[0]) + 100-closet_back_s_diff_[0] - (ref_v_ - closet_back_v_[0]) + 4;
				break;

		// right most lane
		case 2: 
				// cost of staying in same lane base on the distance of vehicle in the front
				cost_[2] += 100-closet_front_s_diff_[2] + (ref_v_ - closet_front_v_[2]);
				// cost of turn left base on any room on the left front and back, and the vehicle behind on the left was traveling faster than me
				cost_[1] += 100-closet_front_s_diff_[1] + (ref_v_ - closet_front_v_[1]) + 100-closet_back_s_diff_[1] - (ref_v_ - closet_back_v_[1]);
				// not supporting to jump 2 lanes, so max cost for right most lane
				cost_[0] += 100;
				break;
	}
}

// check left and right to make sure it is safe to change lane
bool pathPlanning::isChangeLaneSafe(int laneID, int dir){
	// left most lane
	if(laneID == 0){

		// turn left
		if(dir == -1)
			return false;

		// turn right
		if(dir == 1){
			if (closet_front_s_diff_[1]+closet_back_s_diff_[1] > 2*min_safe_dist_)
				return true;
			else
				return false;
		}

		// stay in lane
		else
			return true;
	}

	// middle lane
	if(laneID == 1){

		// turn left
		if(dir == -1){
			if (closet_front_s_diff_[0]+closet_back_s_diff_[0] > 2*min_safe_dist_)
				return true;
			else
				return false;
		}

		// turn right
		if(dir == 1){
			if (closet_front_s_diff_[2]+closet_back_s_diff_[2] > 2*min_safe_dist_)
				return true;
			else
				return false;
		}

		// stay in lane
		else
			return true;
	}

	// right most lane
	else{

		// turn left
		if(dir == -1){
			if (closet_front_s_diff_[1]+closet_back_s_diff_[1] > 2*min_safe_dist_)
				return true;
			else
				return false;
		}

		// turn right
		if(dir == 1)
			return false;

		// stay in lane
		else
			return true;
	}
}

// update states in FWM and determine next state
void pathPlanning::updateFSM(){
	switch(f_) {

		// ==================
		case fsmStateType::keepLane:
		// ==================
			// keepLane state checks lowest cost functions
			if(closet_front_s_diff_[ref_lane_] < min_safe_dist_){
				//reduce speed first
				ref_v_ -= 0.5;
				
				// skip changing lane if it just finished changing lane within a 10th of sec
				if(timer_ > 0){
					timer_ -= dt_*(nextPathSize_-prv_size_);
				}

				//cost of changing lane
				else if(ref_lane_==0 && isChangeLaneSafe(0, 1) && (cost_[0] > cost_[1])){
					f_ = fsmStateType::laneChangeLeft;
					target_lane_ = 1*laneWidth_+2;
					break;
				}
				else if(ref_lane_==2 && isChangeLaneSafe(2, -1) && (cost_[2] > cost_[1])){
					f_ = fsmStateType::laneChangeRight;
					target_lane_ = 1*laneWidth_+2;
					break;
				}
				else if(ref_lane_==1){
					// if both turn left and right are feasible, pick the one with less cost
					if(isChangeLaneSafe(1, -1) && isChangeLaneSafe(1, -1)){
						// change to left if cost of left lane is the lowest
						if(cost_[2] < cost_[0] && cost_[2] < cost_[1]){
							// change lane right
							f_ = fsmStateType::laneChangeRight;
							target_lane_ = 2*laneWidth_+2;
							break;
						}
						// change to right if cost of right lane is the lowest
						else if(cost_[0] < cost_[2] && cost_[0] < cost_[1]){
							// change lane left
							f_ = fsmStateType::laneChangeLeft;
							target_lane_ = 1*laneWidth_+2;
							break;
						}
						// else stay in its lane if cost of its lane is the lowest
					}
				}
			}
			if(timer_ < 0){
				timer_ = 0;
			}
			target_lane_ = ref_lane_;
		break;
		
		// ==================
		case fsmStateType::laneChangeLeft:
		// ==================
			// complete lane change
			if(car_d_ == target_lane_){
				f_ = fsmStateType::keepLane;
				timer_ = 10;
			}
		break;
		
		// ==================
		case fsmStateType::laneChangeRight:
		// ==================
			// complete lane change
			if(car_d_ == target_lane_){
				f_ = fsmStateType::keepLane;
				timer_ = 10;
			}
		break;
	}
	cout << f_ << " state - target_lane=" << target_lane_ << endl;
}


// generate a trajectory base on targeting lane determined by FSM
void pathPlanning::generateTrajectory(vector<double> &previous_path_x, vector<double> &previous_path_y){
	vector<double> ptsx, ptsy;

	// if no previous path exist, use current car points and its previous determine by car_yaw_
	if (prv_size_ < 2){
		std::cout << "=======================================" << endl;
		std::cout << "prv_size_<2 =" << prv_size_ << endl;
		// use two points that make the path tangent to the car
		double car_x2 = car_x_ - cos(car_yaw_);
		double car_y2 = car_y_ - sin(car_yaw_);
		
		ptsx.push_back(car_x2);
		ptsy.push_back(car_y2);
	}
	// otherwise, use the 2nd to last of previous path's end point as starting reference
	else {
		std::cout << "=======================================" << endl;
		std::cout << "prv_size_>2 =" << prv_size_ << endl;
		double prv_x2 = prv_x_[prv_size_-2];
		double prv_y2 = prv_y_[prv_size_-2];
		
		// use two points that make the path tangent to the previous path's end point
		ptsx.push_back(prv_x2);
		ptsy.push_back(prv_y2);
	}

	// push the ref_x_ and ref_y_ as 2nd point, see processSensorData() for their origin
	ptsx.push_back(ref_x_);
	ptsy.push_back(ref_y_);
	std::cout << "1st - ptsx/y.size=" << ptsx.size() << endl;

	// in Frenet add evenly 30m spaced points ahead of the starting reference so the points are not only cover the even distance points
	vector<double> next_wp0 = getXY(car_s_+30, (2+4*target_lane_), map_s_, map_x_, map_y_);
	vector<double> next_wp1 = getXY(car_s_+60, (2+4*target_lane_), map_s_, map_x_, map_y_);
	vector<double> next_wp2 = getXY(car_s_+90, (2+4*target_lane_), map_s_, map_x_, map_y_);
	
	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);
	
	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);
	
	std::cout << "2nd - ptsx/y.size=" << ptsx.size() << endl;

	for(int i=0; i<ptsx.size(); i++){
		//shift car reference angle to 0 degrees to align with local coordinates or in car's prespective
		double shift_x = ptsx[i]-ref_x_;
		double shift_y = ptsy[i]-ref_y_;
		
		ptsx[i] = (shift_x*cos(0-ref_yaw_)) - (shift_y*sin(0-ref_yaw_));
		ptsy[i] = (shift_x*sin(0-ref_yaw_)) + (shift_y*cos(0-ref_yaw_));
		std::cout << "ptsx|ptsy["<<i<<"]=" << ptsx[i] << "|" << ptsy[i] << endl;
	}
	
	// create a spline for fitting
	tk::spline s;
	
	std::cout << "spline - fitting # points=" << ptsx.size() << endl;

	// set (x,y) points to spline
	s.set_points(ptsx, ptsy);
	
	// Since the spline fitting line starts from the last 2 points from the previous path points
	// as a continuation and the rest of the points are newly plotted/picked to predict the new path 
	// for the next 30 meters. With the spline fitting line ready, break the spline evenly to N points with the distance and time interval
	// base on the the speed that we want to travel
	
	// start with all of previous path points from last time
	for(int i=0; i<prv_size_; i++) {
		next_x.push_back(previous_path_x[i]);
		next_y.push_back(previous_path_y[i]);
		std::cout << "prv - next_x["<<i<<"]=" << next_x[i] << endl;
		std::cout << "prv - next_y["<<i<<"]=" << next_y[i] << endl;
	}
	
	// calculate how to break up spline fitting points so that we travel at our desired reference velocity
	double target_x = 30.0;  // meter
	double target_y = s(target_x);
	double target_dist = sqrt((target_x*target_x)+(target_y*target_y));
	double x_add_on = 0;
	
	// fill up the rest of out path planner after filling it with previous points, here we will always output 50 points
	double N = (target_dist/(0.02*maxTravelSpeed_/2.24));  // base on meter/sec
	std::cout << "N|target_dist|ref_v_=" << N << "|" << target_dist << "|" << maxTravelSpeed_ << endl;

	for(int i=1; i<=nextPathSize_-prv_size_; i++){
		double x_point = x_add_on+(target_x/N);
		double y_point = s(x_point);
		
		std::cout << "[i]|x_point|y_point=" << i << "|" << x_point << "|" << y_point << endl;
		
		x_add_on = x_point;
		
		double x_ref = x_point;
		double y_ref = y_point;
		
		// rotate back to normal after rotating it eariler (back to global coordinates)
		x_point = (x_ref*cos(ref_yaw_)) - (y_ref*sin(ref_yaw_));
		y_point = (x_ref*sin(ref_yaw_)) + (y_ref*cos(ref_yaw_));
		
		x_point += ref_x_;
		y_point += ref_y_;
		
		next_x.push_back(x_point);
		next_y.push_back(y_point);
		std::cout << "new - next_x["<<prv_size_-1+i<<"=" << next_x[prv_size_-1+i] << endl;
		std::cout << "new - next_y["<<prv_size_-1+i<<"=" << next_y[prv_size_-1+i] << endl;
	}
}

