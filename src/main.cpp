#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "sensor.h"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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
  
  
  
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      Sensor sensor;
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // get data for our car.
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
      
          int prev_size = previous_path_x.size();

          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          //std::cout << "Actual Car " << car_x << " - " << car_y << std::endl;
          //std::cout << "Sensor Fusion " << sensor_fusion << std::endl;
          
          // input new data from sensor.
          sensor.setNewData(s);
          // Make process of fusion data.
         
          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          vector<double> point_for_spline_x;
          vector<double> point_for_spline_y;

          // Reference x, y, yaw states
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // If previous size is almost empty, use the car as starting reference
          if (prev_size < 2) {
            //car_yaw=0.0; // To avoid spline problem
            // Use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            point_for_spline_x.push_back(prev_car_x);
            point_for_spline_x.push_back(car_x);
            
            point_for_spline_y.push_back(prev_car_y);
            point_for_spline_y.push_back(car_y);
                     
            std::cout << "prev_size < 2 "  <<  "car_yaw "  << car_yaw   <<  "prev_car_x "  << prev_car_x   <<  "prev_car_y "  << prev_car_y   <<  " car_x "  << car_x   <<  " car_y "  << car_y <<  std::endl;
          } else {
            // Use the previous path's endpoint as starting ref
            // Redefine reference state as previous path end point

            // Last point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            // 2nd-to-last point
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            // Use two points that make the path tangent to the path's previous endpoint
            point_for_spline_x.push_back(ref_x_prev);
            point_for_spline_x.push_back(ref_x);

            point_for_spline_y.push_back(ref_y_prev);
            point_for_spline_y.push_back(ref_y);
          }

          // Using Frenet, add 30 m evenly spaced points ahead of the starting reference

          vector<double> next_wp0 = getXY(car_s+40, (2+4*sensor.lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+80, (2+4*sensor.lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+120, (2+4*sensor.lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          point_for_spline_x.push_back(next_wp0[0]);
          point_for_spline_x.push_back(next_wp1[0]);
          point_for_spline_x.push_back(next_wp2[0]);

          point_for_spline_y.push_back(next_wp0[1]);
          point_for_spline_y.push_back(next_wp1[1]);
          point_for_spline_y.push_back(next_wp2[1]);

          for (int i = 0; i < point_for_spline_x.size(); i++) {
            // Shift car reference angle to 0 degrees
            double shift_x = point_for_spline_x[i] - ref_x;
            double shift_y = point_for_spline_y[i] - ref_y;

            point_for_spline_x[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            point_for_spline_y[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }

          // Create a spline called s
          tk::spline s;

          // Set (x,y) points to the spline
          s.set_points(point_for_spline_x, point_for_spline_y);

          // Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all the previous path points from last time
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Compute how to break up spline points so we travel at our desired reference velocity
          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
          double x_add_on = 0;

          // Fill up the rest of the path planner to always output 50 points
          for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
            double N = (target_dist/(.02*sensor.ref_vel/2.24));
            double x_point = x_add_on + (target_x) / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // Rotate back to normal after rotating it earlier
            x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          /*
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }*/
          /*
          double dist_inc = sensor.delta_s;

          for (int i = 0; i < 30; ++i) {
            double next_s=car_s+(i+1)*dist_inc;
            double next_d=6;
            vector<double> xy= getXY(next_s, next_d, map_waypoints_s,map_waypoints_x,map_waypoints_y); 
          
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
*/
          //std::cout << "Send " << sensor.ref_vel << " - "  << std::endl;
          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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