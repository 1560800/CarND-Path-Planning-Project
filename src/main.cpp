#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
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

  int lane = 1; // Middle lane within the Frenet space.
  double ref_vel = 0; // In miles per hour.
  double ref_acc = 0.6; // In miles per hour.

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &ref_acc]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
		  int prev_size = previous_path_x.size();

		  vector<double>ptsx;
		  vector<double>ptsy;

		  if (prev_size > 0)
		  {
			  // For planning purposes, we go to the place in time where the last
			  // predicted path point would be. 
			  car_s = end_path_s;
		  }

		  // Sensor fusion and prediction and trajectory generation.
		  bool too_close = false; // Check the car ahead of me is within 30m
		  bool left_lane_safe = true; // Check the  car in left lane is in range of 30m 
		  bool right_lane_safe = true; // Check the  car in right lane is in range of 30m 

		  int right_lane_number = (lane + 1) <= 2 ? (lane + 1) : 2; 
		  int left_lane_number = (lane - 1) <= 0 ? 0 : (lane - 1);

		  //iterating through sensor fusion to see if there is car ahead
		  for (int i = 0; i < sensor_fusion.size(); i++) {

			  //check other cars in my lane
			  float d = sensor_fusion[i][6];
			  double vx = sensor_fusion[i][3];
			  double vy = sensor_fusion[i][4];
			  double check_speed = sqrt(vx * vx + vy * vy);
			  double check_car_s = sensor_fusion[i][5];
			  double car_lane = getLane(d);

			  //around car's position in future
			  check_car_s += ((double)prev_size * 0.02 * check_speed);

			  if (lane - car_lane == 0) {

				  if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
					  too_close = true;
				  }
			  } //current lane check loop end

			  //check if there is car in safe range of left_lane
			  //only check if there is possibility of taking left lane change
			  if (lane > 0) {
				  //if other car in the loop is in left lane of our car
				  if (lane - car_lane == 1) {
					  //absolute diff as we need to make sure that we have enough gap
					  //between car behind us and ahead of us in left lane
					  float diff = abs(check_car_s - car_s);
					  if (diff < 30) {
						  left_lane_safe = false;
					  }
				  }
			  }

			  if (lane < 2) {
				  //if other car in the loop is in left lane of our car
				  if (lane - car_lane == -1) {
					  //absolute diff as we need to make sure that we have enough gap
					  //between car behind us and ahead of us in left lane
					  float diff = abs(check_car_s - car_s);
					  if (diff < 30) {
						  right_lane_safe = false;
					  }
				  }
			  }

		  } //iterating through sensor fusion to see if there is car ahead loop end

		  //smooth acceleration - tune velocity in incremental mode
		  if (too_close) {
			  ref_vel -= 0.224;
			  if (left_lane_safe && lane > 0 ) {
				  lane = lane - 1;
			  }
			  else if (right_lane_safe && lane < 2) {
				  lane = lane + 1;
			  }
		  }
		  else if (ref_vel < 49.5) {
			  ref_vel += 0.224;
		  }

		  //Reference x, y, and yaw
		  double ref_x = car_x;
		  double ref_y = car_y;
		  double ref_yaw = deg2rad(car_yaw);

		  if (prev_size < 2)
		  {
			  double prev_car_x = car_x - cos(car_yaw);
			  double prev_car_y = car_y - sin(car_yaw);
			  ptsx.push_back(prev_car_x);
			  ptsx.push_back(car_x);
			  ptsy.push_back(prev_car_y);
			  ptsy.push_back(car_y);

		  }
		  else
		  {
			  // Use previous path's end point as starting reference.
			  ref_x = previous_path_x[prev_size - 1];
			  ref_y = previous_path_y[prev_size - 1];

			  double ref_x_prev = previous_path_x[prev_size - 2];
			  double ref_y_prev = previous_path_y[prev_size - 2];
			  ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

			  // Use the previous two points as the begining of the points list
			  // on which to calculate the spline.
			  ptsx.push_back(ref_x_prev);
			  ptsx.push_back(ref_x);

			  ptsy.push_back(ref_y_prev);
			  ptsy.push_back(ref_y);
		  }
	

		// In Frenet coordinate space add evenly spaced by PLANNING_HORIZON_DISTANCE m 
		// three points ahead of the starting reference point.
		vector<double> next_wp0 = getXY(
			car_s + 30 * 1,
			(2 + 4 * lane),
			map_waypoints_s,
			map_waypoints_x,
			map_waypoints_y
		);
		vector<double> next_wp1 = getXY(
			car_s + 30 * 2,
			2 + 4 * lane,
			map_waypoints_s,
			map_waypoints_x,
			map_waypoints_y
		);
		vector<double> next_wp2 = getXY(
			car_s + 30 * 3,
			2 + 4 * lane,
			map_waypoints_s,
			map_waypoints_x,
			map_waypoints_y
		);

		ptsx.push_back(next_wp0[0]);
		ptsx.push_back(next_wp1[0]);
		ptsx.push_back(next_wp2[0]);

		ptsy.push_back(next_wp0[1]);
		ptsy.push_back(next_wp1[1]);
		ptsy.push_back(next_wp2[1]);

		// Shift reference points to ego-car own coordinate system.
		// Shift car heading to 0 degrees (for everything being in ego coodinates).
		for (int i = 0; i < ptsx.size(); ++i) {
			double shift_x = ptsx[i] - ref_x;
			double shift_y = ptsy[i] - ref_y;

			ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
			ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
		}

		tk::spline s; // Spline object.
		// Calculate the resulting spline in ego-car coordinate system.
		// This way it minimizes the instant velocity and acceleration.
		s.set_points(ptsx, ptsy);

		vector<double> next_x_vals;
		vector<double> next_y_vals;

		for (int i = 0; i < prev_size; ++i) {
			next_x_vals.push_back(previous_path_x[i]);
			next_y_vals.push_back(previous_path_y[i]);
		}

		// Calculate how to break up spline points so that we travel at our
		// desired reference velocity.
		double target_x = 30.0;
		double target_y = s(target_x);
		double target_dist = sqrt(target_x * target_x + target_y * target_y);

		double x_add_on = 0;

		// Fill up the rest of our path planner after filling it with prevoius
		// points, here we will always output PLANNING_NUM_INTEVALS points
		for (int i = 1; i <= 50 - previous_path_x.size(); i++) { //----
			double N = (target_dist / (0.02 * ref_vel / 2.24));

			double x_point = x_add_on + (target_x) / N;
			double y_point = s(x_point);

			x_add_on = x_point;

			double x_ref = x_point ;
			double y_ref = y_point ;

			// Rotate back to global coordinate system after rotating it earlier.
			x_point = (x_ref * cos(ref_yaw ) - y_ref * sin(ref_yaw ));
			y_point = (x_ref * sin(ref_yaw ) + y_ref * cos(ref_yaw ));

			x_point += ref_x;
			y_point += ref_y;

			next_x_vals.push_back(x_point);
			next_y_vals.push_back(y_point);
		}


          json msgJson;


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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