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
		closest_front_s_diff_[i] = 999.0;
		closest_back_s_diff_[i]  = 999.0;
		closest_front_id_[i] = 0;
		closest_back_id_[i]  = 0;
		closest_front_v_[i]  = 0.0;
		closest_back_v_[i]   = 0.0;
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
	prv_size_  = min(int(previous_path_x.size()), 10);  // cap the previous point to be reused at the 10th point for smoothing
	
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
		ref_y_ = prev_y1;
		ref_s_ = prev_sd[0];
		ref_d_ = prev_sd[1];
		ref_yaw_ = prev_yaw;
		ref_v_ = prev_v_mps;
		if(debug) cout << "prev_* ------>" << endl;
		if(debug) cout << "x1|y1| x2|y2| yaw|s|d|v_mps|v MPH = " << ref_x_ <<"|"<< ref_y_ <<"|| "<< prev_x2 <<"|"<< prev_y2 <<"|| "<< ref_yaw_ <<"|| "<< ref_s_ <<"|"<< ref_d_ <<"|| "<< ref_v_ <<"|| "<< ref_v_/MILE_PER_HOUR_2_METER_PER_SEC<< endl;
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
//		if(ref_s_ < vehicle_s_at_prev_path_end){
		if(car_s_ <= vehicle_s){
			// check if vehicle is closer than previous recorded in its lane
			if(s_diff < closest_front_s_diff_[vehicle_lane]){
				closest_front_s_diff_[vehicle_lane] = s_diff;
				closest_front_id_[vehicle_lane] = vehicle_id;  // just in case if we need it for look up
				closest_front_v_[vehicle_lane] = vehicle_v;
			}
		}
		// vehicle is from the back 
		else {
			// check if vehicle is closer than previous recorded in its lane
			if(s_diff < closest_back_s_diff_[vehicle_lane]){
			//if(car_s_-vehicle_s < closest_back_s_diff_[vehicle_lane]){
				closest_back_s_diff_[vehicle_lane] = s_diff;
				//closest_back_s_diff_[vehicle_lane] = fabs(car_s_-vehicle_s);
				closest_back_id_[vehicle_lane] = vehicle_id;  // just in case if we need it for look up
				closest_back_v_[vehicle_lane] = vehicle_v;
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
			// cost of staying in same lane base on the distance of vehicle in the front
			cost_[0] += closest_front_s_diff_[0] > laneVisibleDist_ ? 0 : 
						(ref_v_ - closest_front_v_[0])==0 ? 100 : 
						max(0.0, min(100.0 - exp(closest_front_s_diff_[0]/(ref_v_ - closest_front_v_[0])), 100.0));
			// cost of turn right base on any room on the right front
			cost_[1] += closest_front_s_diff_[1] > laneVisibleDist_ ? 0 : 
						(ref_v_ - closest_front_v_[1])==0 ? 100 : 
						max(0.0, min(100.0 - exp(closest_front_s_diff_[1]/(ref_v_ - closest_front_v_[1])), 100.0));
			// not supporting to jump 2 lanes, so max cost for right most lane
			cost_[2] += 100;
			break;

		// middle lane
		case 1: 
			// cost of turn left base on any room on the left front
			cost_[0] += closest_front_s_diff_[0] > laneVisibleDist_ ? 0 : 
						(ref_v_ - closest_front_v_[0])==0 ? 100 : 
						max(0.0, min(100.0 - exp(closest_front_s_diff_[0]/(ref_v_ - closest_front_v_[0])), 100.0));
			// cost of staying in same lane base on the distance of vehicle in the front
			cost_[1] += closest_front_s_diff_[1] > laneVisibleDist_ ? 0 : 
						(ref_v_ - closest_front_v_[1])==0 ? 100 : 
						max(0.0, min(100.0 - exp(closest_front_s_diff_[1]/(ref_v_ - closest_front_v_[1])), 100.0));
			// cost of turn right base on any room on the right front
			// left turn to the fastest lane is preferrable, thus add 2 to right cost function
			cost_[2] += closest_front_s_diff_[2] > laneVisibleDist_ ? 0 : 
						(ref_v_ - closest_front_v_[2])==0 ? 100 : 
						max(0.0, min(100.0 - exp(closest_front_s_diff_[2]/(ref_v_ - closest_front_v_[2]))+2, 100.0));
			break;

		// right most lane
		case 2: 
			// not supporting to jump 2 lanes, so max cost for right most lane
			cost_[0] += 100;
			// cost of turn left base on any room on the left front
			cost_[1] += closest_front_s_diff_[1] > laneVisibleDist_ ? 0 : 
						(ref_v_ - closest_front_v_[1])==0 ? 100 : 
						max(0.0, min(100.0 - exp(closest_front_s_diff_[1]/(ref_v_ - closest_front_v_[1])), 100.0));
			// cost of staying in same lane base on the distance of vehicle in the front
			cost_[2] += closest_front_s_diff_[2] > laneVisibleDist_ ? 0 : 
						(ref_v_ - closest_front_v_[2])==0 ? 100 : 
						max(0.0, min(100.0 - exp(closest_front_s_diff_[2]/(ref_v_ - closest_front_v_[2])), 100.0));
			break;
	}
}


// check left and right to make sure it is safe to change lane
// input:
// laneID - 0=left most lane, 1=middle lane, 2=right most lane
// dir    - 0=stay in lane,   1=turn right, -1=turn left
bool pathPlanning::isChangeLaneSafe(int laneID, int dir){
	// check distance of the vehicle in front has enough couhsion first
	if(closest_front_s_diff_[ref_lane_] > min_safe_dist_front_){
		
		// left most lane
		if(laneID == 0){

			// turn left
			if(dir == -1)
				return false;

			// turn right
			if(dir == 1){
//				if (closest_front_s_diff_[1] > min_lane_change_dist_ && closest_back_s_diff_[1] > min_safe_dist_back_)
				if (closest_front_s_diff_[1] > min_safe_dist_front_ && closest_back_s_diff_[1] > min_safe_dist_back_)
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
				if (closest_front_s_diff_[0] > min_safe_dist_front_ && closest_back_s_diff_[0] > min_safe_dist_back_)
					return true;
				else
					return false;
			}
 
			// turn right
			if(dir == 1){
				if (closest_front_s_diff_[2] > min_safe_dist_front_ && closest_back_s_diff_[2] > min_safe_dist_back_)
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
				if (closest_front_s_diff_[1] > min_safe_dist_front_ && closest_back_s_diff_[1] > min_safe_dist_back_)
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
	// too close to the vehicle in the front
	else {
		if(debug) cout << "WARNING!!!! Too close to vehicle in front. No change of lane YET!!" << endl; 
		return false;
	}
}


void pathPlanning::print_closest_all(){
	std::cout << "min_lane_change_dist_=" << min_lane_change_dist_ << endl;
	std::cout << "ref_lane_=" << ref_lane_ << endl;
	for(int i=0; i<laneSize_; i++)
		std::cout << "closest_front_s_diff_[" << i << "]=" << closest_front_s_diff_[i] << " id=" <<  closest_front_id_[i] << " at " <<  closest_front_v_[i] << "mps" << endl;
	for(int i=0; i<laneSize_; i++)
		std::cout << "closest_back_s_diff_[" << i << "]=" << closest_back_s_diff_[i] << " id=" <<  closest_back_id_[i] << " at " <<  closest_back_v_[i] << "mps" << endl;
	std::cout << "isChangeLaneSafe(0, 1)=   " << isChangeLaneSafe(0, 1) << endl;
	std::cout << "isChangeLaneSafe(2,-1)=   " << isChangeLaneSafe(2,-1) << endl;
	std::cout << "isChangeLaneSafe(1,-1)=   " << isChangeLaneSafe(1,-1) << endl;
	std::cout << "isChangeLaneSafe(1, 1)=   " << isChangeLaneSafe(1, 1) << endl;
	for(int i=0; i<laneSize_; i++)
		std::cout << "   cost_[" << i << "]=" << cost_[i] << endl;
}

// update states in FWM and determine next state
void pathPlanning::updateFSM(){
	// set targeting speed to either match the speed limit or the vehicle in front of target lane 
	double target_v = maxTravelSpeed_*MILE_PER_HOUR_2_METER_PER_SEC;

	switch(f_) {

		// ==================
		case fsmStateType::keepLane:
		// ==================
			if(debug) print_closest_all();
			// consider change lane if gap between car in front is closer than minium lane change distance
			if(closest_front_s_diff_[ref_lane_] < min_lane_change_dist_){
				
				// reduce speed first to match the speed of car in front
				ref_v_ = max(ref_v_-0.6, closest_front_v_[ref_lane_]-0.6);
				if(debug) cout << "---------- SSS: REDUC -0.6 mps to avoid hitting car in front " << ref_v_ << "mps" << endl;

				// skip changing lane if it just finished changing lane within 10th of sec
				if(timer_ == 0){
					//cost of changing lane
					if(ref_lane_==0 && isChangeLaneSafe(0, 1) && (cost_[0] > cost_[1])){
						f_ = fsmStateType::laneChangeRight;
						target_lane_ = 1;
						if(debug) cout << "SSS: change lane from 0 to RIGHT to " << target_lane_<< endl;
						break;
					}
					else if(ref_lane_==2 && isChangeLaneSafe(2, -1) && (cost_[2] > cost_[1])){
						f_ = fsmStateType::laneChangeLeft;
						target_lane_ = 1;
						if(debug) cout << "SSS: change lane from 2 to LEFT to " << target_lane_<< endl;
						break;
					}
					else if(ref_lane_==1){
						// change to left if cost of left lane is the lowest and if it is feasible
						if(cost_[0] <= cost_[2] && cost_[0] < cost_[1] && isChangeLaneSafe(1, -1)){
							// change lane left
							f_ = fsmStateType::laneChangeLeft;
							target_lane_ = 0;
							if(debug) cout << "SSS: change lane from 1 to LEFT to " << target_lane_<< endl;
							break;
						}
						// change to right if cost of right lane is the lowest and if it is feasible
						else if(cost_[2] <= cost_[0] && cost_[2] < cost_[1] && isChangeLaneSafe(1, 1)){
							// change lane right
							f_ = fsmStateType::laneChangeRight;
							target_lane_ = 2;
							if(debug) cout << "SSS: change lane from 1 to RIGHT to " << target_lane_<< endl;
							break;
						}
						// else stay in its lane if cost of its lane is the lowest
					}
				}
			}
			else if(ref_v_ < maxTravelSpeed_*MILE_PER_HOUR_2_METER_PER_SEC){
				if(debug) cout << "---------- SSS: ACCEL +0.8 mps to catch up speed" << endl;
				ref_v_ = min(ref_v_+0.8, maxTravelSpeed_*MILE_PER_HOUR_2_METER_PER_SEC);
			}else if (ref_v_ > maxTravelSpeed_*MILE_PER_HOUR_2_METER_PER_SEC){
				if(debug) cout << "---------- SSS: REDUC -0.6 mps to stay under sped lmit" << endl;
				ref_v_ = max(ref_v_-0.6, maxTravelSpeed_*MILE_PER_HOUR_2_METER_PER_SEC);
			}

			if(timer_ < 0){
				timer_ = 0;
				if(debug) cout << "SSS: reset timer" << endl;
			} else if(timer_ > 0){
				if(debug) cout << "SSS: decrement timer" << endl;
				timer_ -= dt_*(nextPathSize_-prv_size_);
			}
			target_lane_ = ref_lane_;
		break;
		
		// ==================
		case fsmStateType::laneChangeLeft:
		// ==================
			if(debug) print_closest_all();
			// lane change completed
			if(car_lane_ == target_lane_){
				f_ = fsmStateType::keepLane;
				// start timer to settle in to new lane before another lane change
				timer_ = 2;
			}
			
			// set targeting speed to either match the speed limit or the vehicle in front of target lane 
			//double target_v;
			//target_v = maxTravelSpeed_*MILE_PER_HOUR_2_METER_PER_SEC;
			if(closest_front_v_[target_lane_] > 0 && closest_front_v_[target_lane_] < target_v){
				target_v = closest_front_v_[target_lane_];
				if(debug) cout << "<<<<------ LLL: Slow down to match new lane speed" << target_v << endl;
			}

			// adjust speed
			if(ref_v_ < target_v){
				// purposely lower acceleration to +0.6 from +0.8 to reduce jerk when changing lane at the curve
				ref_v_ = min(ref_v_+0.6, target_v);
				if(debug) cout << "<<<<------ LLL: ACCEL +0.6 mps to catch up speed to " << ref_v_/MILE_PER_HOUR_2_METER_PER_SEC << " MPH" << endl;
			}else{
				// purposely increase decceleration to -0.6 from -0.8 to reduce jerk when changing lane at the curve
				ref_v_ = max(ref_v_-0.8, target_v);
				if(debug) cout << "<<<<------ LLL: REDUC -0.8 mps to stay under speed lmit " << ref_v_/MILE_PER_HOUR_2_METER_PER_SEC << " MPH" << endl;
			}
		break;
		
		// ==================
		case fsmStateType::laneChangeRight:
		// ==================
			if(debug) print_closest_all();
			// complete lane change
			if(car_lane_ == target_lane_){
				f_ = fsmStateType::keepLane;
				// start timer to settle in to new lane before another lane change
				timer_ = 2;
			}
			// set targeting speed to either match the speed limit or the vehicle in front of target lane 
			//double target_v;
			//target_v = maxTravelSpeed_*MILE_PER_HOUR_2_METER_PER_SEC;
			if(closest_front_v_[target_lane_] > 0 && closest_front_v_[target_lane_] < target_v){
				target_v = closest_front_v_[target_lane_];
				if(debug) cout << "------>>>> RRR: Slow down to match new lane speed" << target_v << endl;
			}
			if(ref_v_ < target_v){
				// purposely lower acceleration to +0.6 from +0.8 to reduce jerk when changing lane at the curve
				ref_v_ = min(ref_v_+0.6, target_v);
				if(debug) cout << "------>>>> RRR: ACCEL +0.6 mps to catch up speed" << ref_v_/MILE_PER_HOUR_2_METER_PER_SEC << " MPH" << endl;
			}else{
				// purposely increase decceleration to -0.6 from -0.8 to reduce jerk when changing lane at the curve
				ref_v_ = max(ref_v_-0.8, target_v);
				if(debug) cout << "------>>>> RRR: REDUC -0.8 mps to stay under sped lmit" << ref_v_/MILE_PER_HOUR_2_METER_PER_SEC << " MPH" << endl;
			}
		break;
	}
	if(debug) cout << f_ << " state - target_lane=" << target_lane_ << endl;
}


// generate a trajectory base on targeting lane determined by FSM
void pathPlanning::generateTrajectory(vector<double> &previous_path_x, vector<double> &previous_path_y){
	vector<double> ptsx, ptsy;
	next_x.clear();
	next_y.clear();

	// if no previous path exist, use current car points and its previous determine by car_yaw_
	if (prv_size_ < 2){
		if(debug) cout << "=======================================" << endl;
		if(debug) cout << "ref size <2 =" << prv_size_ << endl;
		if(debug) cout << "prv_size_<2 =" << previous_path_x.size() << endl;
		// use two points that make the path tangent to the car
		double car_x2 = car_x_ - cos(car_yaw_);
		double car_y2 = car_y_ - sin(car_yaw_);
		
		ptsx.push_back(car_x2);
		ptsy.push_back(car_y2);
		if(debug) cout << "curent_* ------>" << endl;
		if(debug) cout << "x1|y1| x2|y2| yaw|s|d|v mps|v MPH = " << car_x_ <<"|"<< car_y_ <<"|| "<< car_x2 <<"|"<< car_y2 <<"|| "<< car_yaw_ <<"|| "<< car_s_ <<"|"<< car_d_ <<"|| "<< car_v_mps_ <<"|| "<< car_speed_ << endl;
	}
	// otherwise, use the 2nd to last of previous path's end point as starting reference
	else {
		if(debug) cout << "=======================================" << endl;
		if(debug) cout << "ref size >2 =" << prv_size_ << endl;
		if(debug) cout << "prv_size_>2 =" << previous_path_x.size() << endl;
		double prv_x2 = prv_x_[prv_size_-2];
		double prv_y2 = prv_y_[prv_size_-2];
		
		// use two points that make the path tangent to the previous path's end point
		ptsx.push_back(prv_x2);
		ptsy.push_back(prv_y2);
		if(debug) cout << "prev_* ------>" << endl;
		if(debug) cout << "x1|y1| x2|y2| yaw|s|d|v mps|v MPH = " << ref_x_ <<"|"<< ref_y_ <<"|| "<< prv_x2 <<"|"<< prv_y2 <<"|| "<< ref_yaw_ <<"|| "<< ref_s_ <<"|"<< ref_d_ <<"|| "<< ref_v_<<"|| "<< ref_v_/MILE_PER_HOUR_2_METER_PER_SEC << endl;
	}

	// push the ref_x_ and ref_y_ as 2nd point, see processSensorData() for their origin
	ptsx.push_back(ref_x_);
	ptsy.push_back(ref_y_);
	
	// in Frenet add evenly 30m spaced points ahead of the starting reference so the points are not only
	// cover the even distance points. original 30m apart, curently increase to 50m apart to further
	// reduce jerk during lane change by sketching out the spline fit line
	vector<double> next_wp0 = getXY(car_s_+50,  (2+laneWidth_*target_lane_), map_s_, map_x_, map_y_);
	vector<double> next_wp1 = getXY(car_s_+100, (2+laneWidth_*target_lane_), map_s_, map_x_, map_y_);
	vector<double> next_wp2 = getXY(car_s_+150, (2+laneWidth_*target_lane_), map_s_, map_x_, map_y_);
	
	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);
	
	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);
	
	for(int i=0; i<ptsx.size(); i++){
		//shift car reference angle to 0 degrees to align with local coordinates or in car's prespective
		double shift_x = ptsx[i]-ref_x_;
		double shift_y = ptsy[i]-ref_y_;
		
		ptsx[i] = (shift_x*cos(0-ref_yaw_)) - (shift_y*sin(0-ref_yaw_));
		ptsy[i] = (shift_x*sin(0-ref_yaw_)) + (shift_y*cos(0-ref_yaw_));
	}
	
	// create a spline for fitting
	tk::spline s;
	
	// set (x,y) points to spline
	s.set_points(ptsx, ptsy);
	
	// Since the spline fitting line starts from the last 2 points from the previous path points
	// as a continuation and the rest of the points are newly plotted/picked to predict the new path 
	// for the next 30 meters. With the spline fitting line ready, break the spline evenly to N points 
	// with the distance and time interval base on the the speed that we want to travel
	// start with all of previous path points from last time
	for(int i=0; i<prv_size_; i++) {
		next_x.push_back(previous_path_x[i]);
		next_y.push_back(previous_path_y[i]);
	}
	
	// calculate how to break up spline fitting points so that we travel at our desired reference velocity
	double target_x = 30.0;  // meter
	double target_y = s(target_x);
	double target_dist = sqrt((target_x*target_x)+(target_y*target_y));
	double x_add_on = 0;
	
	// fill up the rest of out path planner after filling it with previous points, here we will always output 50 points
	double N = (target_dist/(0.02*ref_v_));  // ref_v base on meter/sec

	for(int i=1; i<=nextPathSize_-prv_size_; i++){
		double x_point = x_add_on+(target_x/N);
		double y_point = s(x_point);
		
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
	}
	if (debug && next_x.size() < 50){
		cout << "WARNING !!!!! number points of next_xy less than 50 = " << next_x.size() << endl;
		cout << "WARNING !!!!! number points of next_xy less than 50 = " << next_x.size() << endl;
		cout << "WARNING !!!!! number points of next_xy less than 50 = " << next_x.size() << endl;
	}
}

