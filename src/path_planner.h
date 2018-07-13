#ifndef PATHPLANNING_H
#define PATHPLANNING_H


using namespace std;

class pathPlanning
{
	public:
		// constructor
		pathPlanning() {}
		
		// destructor
		virtual ~pathPlanning() {}
		
		// new data path array in global coordinates 
		vector<double> next_x;
		vector<double> next_y;

		// determine lane position
		int getLane(double d);
		
		// initialize all cost variables before processing
		void initPathPlanning(vector<double> &map_x, vector<double> &map_y, vector<double> &map_s, vector<double> &map_dx, vector<double> &map_dy);
		
		// process sensor data fron sensor fusion
		void processSensorData(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, vector<double> &previous_path_x, vector<double> &previous_path_y, double end_path_s, double end_path_d, vector<vector<double>> sensor_fusion);
		
		// check if it is safe to change lane
		bool isChangeLaneSafe(int laneID, int dir);
		
		// update states in FWM, determine next state and generate target_lane
		void updateFSM();
		
		// printing debug message
		void print_closet_all();

		// generate a trajectory base on targeting lane determined by FSM
		void generateTrajectory(vector<double> &previous_path_x, vector<double> &previous_path_y);

	private:
		const double MILE_PER_HOUR_2_METER_PER_SEC = 0.44704;

		// max path size, default to 50
		int nextPathSize_ = 50;
		
		// time between each next path points
		double dt_ = 0.02;
		
		// timer for stablizing to keepLane state after a changeLane state
		double timer_ = 0.5;
		
		// max desire speed
		double maxTravelSpeed_ = 49.2;
		
		// map waypoints data
		vector<double> map_x_;
		vector<double> map_y_;
		vector<double> map_s_;
		vector<double> map_dx_;
		vector<double> map_dy_;
		
		// lane info
		int laneSize_ = 3;
		int laneWidth_ = 4.0;	// in meters
		double laneSpeedLimit_ = 50.0; // in miles per hour
		double laneVisibleDist_ = 50.0;  // in meters
		double min_safe_dist_ = 18.0; // in meters (22.35mps ~ 50MPH), so 50 meters are more than double
		double min_safe_dist_back_ = 5.0; // in meters (22.35mps ~ 50MPH), so 50 meters are more than double
		
		// current car info
		double car_x_;
		double car_y_;
		double car_s_;
		double car_d_;
		double car_yaw_;
		double car_speed_;  // in MPH
		double car_v_mps_;  // in meter per second
		int car_lane_;   // current car lane
		//vector<vector<double>> car_sensor_fusion_;  // other cars array through car sensors
													// car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates, 
		// previous path data
		vector<double> prv_x_;
		vector<double> prv_y_;
		double prv_end_s_;
		double prv_end_d_;
		int prv_size_;      // previous path array size

		// reference variables for next path calculation. starts from the last 2 points of previous path to reserve continuation
		double ref_x_;
		double ref_y_;
		double ref_s_;
		double ref_d_;
		double ref_yaw_;
		double ref_v_;
		int ref_lane_;
		
		// cost variables for each lane
		vector<double> cost_ = {0.0, 0.0, 0.0};

		// closest distance to the vehicle ahead for each lane
		vector<double> closet_front_s_diff_ = {999.0, 999.0, 999.0};
		vector<double> closet_back_s_diff_  = {999.0, 999.0, 999.0};
		vector<int>    closet_front_id_ = {0, 0, 0};
		vector<int>    closet_back_id_  = {0, 0, 0};
		vector<double> closet_front_v_ = {0.0, 0.0, 0.0};
		vector<double> closet_back_v_  = {0.0, 0.0, 0.0};
		
		enum fsmStateType {
			keepLane,
			laneChangeLeft,
			laneChangeRight
		};
		
		fsmStateType f_ = keepLane;
		int target_lane_;
};

#endif