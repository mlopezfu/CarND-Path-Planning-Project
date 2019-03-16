#include "sensor.h"
#include <cmath> 
#include "json.hpp"
//#include <vector>




// for convenience
using namespace std;
using nlohmann::json;

double Sensor::prior_distance_ahead=1000;
double Sensor::delta_s=0;
int Sensor::lane =1; //Begin at lane 1
int Sensor::best_lane=1;//Begin at lane 1
//int Sensor::best_lane_sr=1;//Begin at lane 1
double Sensor::max_speed=49.5;
double Sensor::front_reference_speed=49.5;
bool Sensor::try_to_change=false;
	// Reference velocity to target
double Sensor::ref_vel = 0; // mph
Sensor::Sensor() {
	
}

void Sensor::setNewData(string jsonData) {
	debug=false;
	bool changing_lane=false;
	auto j = json::parse(jsonData);
	
	// Main car's localization Data
	double car_x = j[1]["x"];
	double car_y = j[1]["y"];
	double car_s = j[1]["s"];
	double car_d = j[1]["d"];
	double car_yaw = j[1]["yaw"];
	double car_speed = j[1]["speed"];

	//if (debug) std::cout << "Actual Car " << car_x << " - " << car_y<< " - " << car_s << " - " << car_d << std::endl;

	// Previous path data given to the Planner
	auto previous_path_x = j[1]["previous_path_x"];
	auto previous_path_y = j[1]["previous_path_y"];
	int prev_size = previous_path_x.size();
	// Previous path's end s and d values 
	double end_path_s = j[1]["end_path_s"];
	double end_path_d = j[1]["end_path_d"];
	
	//if (debug) std::cout << "prev_size " << prev_size  << std::endl;
	// Sensor Fusion Data, a list of all other cars on the same side 
	//   of the road.
	auto sensor_fusion = j[1]["sensor_fusion"];
	/*
	The data format for each car is: 
	[ id, x, y, vx, vy, s, d]. 
	The id is a unique identifier for that car. 
	The x, y values are in global map coordinates. 
	The vx, vy values are the velocity components, also in reference to the global map. 
	Finally s and d are the Frenet coordinates for that car.

	The vx, vy values can be useful for predicting where the cars will be in the future. 
	For instance, if you were to assume that the tracked car kept moving along the road, 
	then its future predicted Frenet s value will be its current s value plus its (transformed) 
	total velocity (m/s) multiplied by the time elapsed into the future (s).
	*/
	if (lane!=carLane(car_d))
	{
		//std::cout << "changing from lane  " << carLane(car_d) << " to " << lane << std::endl;
		changing_lane=true;
	}



	
	// Lane identifiers for other cars
	bool too_close = false;
	bool car_left = false;
	bool car_right = false;
	
	double front_reference_distance=40;
	int front_gap = 40; 
	int lateral_gap=20;
	int lane_cars_front_sr[3]={0,0,0};
	int lane_cars_front_lr[3]={0,0,0};
	// For each car in the sensor fusion.
	for (int i = 0; i < sensor_fusion.size(); i++) {
	
		double d = sensor_fusion[i][6];

		// Identify the lane of the car
		int car_lane=carLane(d);

		double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
		double check_speed = sqrt(vx*vx + vy*vy);
		double check_car_s = sensor_fusion[i][5];

		// If using previous points can project an s value outwards in time
		// (What position we will be in in the future)
		// check s values greater than ours and s gap
		double future_check_car_s =check_car_s+((double)prev_size*0.02*check_speed);

		double distance=abs(check_car_s - car_s);
		if (distance>0 and distance<100)
		{
			//if (debug) std::cout << "Distance " << distance << std::endl;
			lane_cars_front_lr[car_lane]++;
			if (distance>0  and distance<50)
				lane_cars_front_sr[car_lane]++;
		}

		// check occupancy
		if (car_lane == lane) {
			
			// Another car is ahead
			if ((check_car_s > car_s) && ((check_car_s - car_s) < front_gap))
			{
				too_close |= true;
				
				if (front_reference_distance>distance)
				{
					front_reference_distance=distance;
					front_reference_speed=check_speed*1.609;
					try_to_change=true;
				}

			}
			//too_close |= (check_car_s > car_s) && ((check_car_s - car_s) < gap);

			//too_close_back |= (check_car_s < car_s) && ((car_s - check_car_s) < gap);
			//if (too_close_back) std::cout << "Too close " << i << " - " << (car_s - check_car_s) << " - " << car_s << " - " << check_car_s << std::endl;
			//if (too_close_ahead) std::cout << "Too close " << i << " - " << (check_car_s - car_s)  << " - " << car_s << " - " << check_car_s << std::endl;
		} else if (car_lane - lane == 1) {
			//if (check_car_s-lateral_gap<car_s && car_s<future_check_car_s+lateral_gap)
			if (check_car_s-lateral_gap<car_s && car_s<future_check_car_s)
			{
				if (debug) std::cout << "right car_s " << car_s << " check_car_s " << check_car_s   << " future_check_car_s " <<  future_check_car_s   << std::endl;	
				car_right |= true;//((car_s - lateral_gap) < future_check_car_s) && ((car_s + lateral_gap) > future_check_car_s);					
			}	
			// Another car is to t66 he right
			
			//if (debug) std::cout << "Right " << i << std::endl;
		} else if (lane - car_lane == 1) {
			//if (check_car_s-lateral_gap<car_s && car_s<future_check_car_s+lateral_gap)
			if (check_car_s-lateral_gap<car_s && car_s<future_check_car_s)
			{
				if (debug) std::cout << "left car_s " << car_s << " check_car_s " << check_car_s   << " future_check_car_s " <<  future_check_car_s   << std::endl;
				car_left |= true;
			}	
			// Another car is to the left
			//car_left |= ((car_s - lateral_gap) < future_check_car_s) && ((car_s + lateral_gap) > future_check_car_s);
			//if (debug) std::cout << "Left " << i << std::endl;
		}
	}
	int temp_best_lane_lr=0;
	int temp_best_lane_sr=0;
	for (int i=1;i<3;i++)
	{
		if (lane_cars_front_lr[temp_best_lane_lr]>lane_cars_front_lr[i])
		{
			temp_best_lane_lr=i;
		}
		if (lane_cars_front_sr[temp_best_lane_sr]>lane_cars_front_sr[i])
		{
			temp_best_lane_sr=i;
		}
	}
	if (temp_best_lane_lr==lane)
	{
		if (too_close)
			best_lane=temp_best_lane_sr;
		else
			best_lane=lane;
	}
	else if (abs(temp_best_lane_lr-lane)>1)
	{// If there are more than 1 lane to best
		if (temp_best_lane_lr>lane)
			best_lane=lane+1;
		else
			best_lane=lane-1;

		try_to_change=true;
	}
	else
	{
		
		best_lane=temp_best_lane_lr;
		
		
		try_to_change=true;
	}
	if (debug) std::cout << "lane_cars_front_lr " << lane_cars_front_lr[0] << " " << lane_cars_front_lr[1] << " " << lane_cars_front_lr[2] << " Lane " << lane << " - best_lane :" << best_lane <<  std::endl;
	if (debug) std::cout << "lane_cars_front_sr " << lane_cars_front_sr[0] << " " << lane_cars_front_sr[1] << " " << lane_cars_front_sr[2] << " Lane " << lane << " - car_right :" << car_right << " - car_left :" << car_left <<  std::endl;
	double acc = 0.224;
	double brake_acc = 0.124;
	max_speed = min(49.5,front_reference_speed);
	double max_speed_for_lane_change = min(40.0,front_reference_speed);
	if (try_to_change && !changing_lane) {
		// A car is ahead
		// Decide to shift lanes or slow down
		if ( ((!car_right && best_lane>lane) || (!car_left && best_lane<lane)) && ref_vel<max_speed_for_lane_change)
		{
			if (lane==best_lane)
			{	if(lane>0)
					lane--;
				else if(lane<2)
					lane++;
			}
			else
				lane=best_lane;
			try_to_change=false;
		}
		else {
			if (ref_vel>max_speed || ref_vel>max_speed_for_lane_change)
			{
				// Nowhere to shift -> slow down
				ref_vel -= brake_acc;
				ref_vel=abs(ref_vel); // Avoid negative
			}
		}
	} 
	front_reference_speed+=5; //Increments front speed.
		
	if (ref_vel < max_speed) {
		// No car ahead AND we are below the speed limit -> speed limit
		ref_vel += acc;
	}
}

int Sensor::carLane(float d)
{
	int car_lane=0;
	// Each lane is 4 meters wide.
	if (d >= 0 && d < 4) {
		car_lane = 0;
	} else if (d >= 4 && d < 8) {
		car_lane = 1;
	} else if (d >= 8 && d <= 12) {
		car_lane = 2;
	} else {
		//continue;
	}
	return car_lane;
}
//Sensor::~Sensor() {}
