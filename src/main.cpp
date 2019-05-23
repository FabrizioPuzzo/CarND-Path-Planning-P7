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

  int lane = 1;
  double ref_vel = 0.0;

  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event
                // set needed constants ----------
                const float LANE_WIDTH = 4.0; // in meters
                const float MAX_SPEED = 49.5; // in mph
                const float MAX_ACCELERATION = .224; // in mph^2 (~ 5 m/s^2)
                const int MAX_HIGHWAY_LANES = 3;
                const int SAFETY_GAP_FRONT = 30; // in meters
                const int SAFETY_GAP_BEHIND = 10; // in meters


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
                      vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

                      // get number of waypoints
                      int prev_size = previous_path_x.size();

                      // avoid collisions
                      if (prev_size > 0) {
                        car_s = end_path_s;
                      }

                      // initialize boolean for situation evaluation
                      bool unsafe2turn_left = false;
                      bool unsafe2go_ahead = false;
                      bool unsafe2turn_right = false;
                      double speed_ahead;

                      // iterate through every detected car and assign values to the 
                      // situation booleans based on the position and behaviour of the other cars
                      for (int i = 0; i < sensor_fusion.size(); i++) {
                        
                        // load d value
                        float d = sensor_fusion[i][6];
                        int current_car_lane;

                        // check current cars position on the road (d) and assign lane
                        for (int lane_i = 0; lane_i < MAX_HIGHWAY_LANES; lane_i++) {
                          if (d > lane_i * LANE_WIDTH && d < (lane_i + 1) * LANE_WIDTH) {
                            current_car_lane = lane_i;
                            break;
                          }
                        }

                        // check cars speed
                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double check_speed = sqrt(vx * vx + vy * vy);
                        double check_car_s = sensor_fusion[i][5];

                        speed_ahead = check_speed;

                        // calculate cars position (s)
                        check_car_s += ((double)prev_size * .02 * check_speed);

                        // check when lane is unsafe to make a move by assigning values to the situation booleans based
                        // on the position of the other cars
                        if (current_car_lane == lane - 1) {
                          unsafe2turn_left = unsafe2turn_left || 
                            (check_car_s - car_s > -SAFETY_GAP_BEHIND && check_car_s - car_s < SAFETY_GAP_FRONT);
                        } else if (current_car_lane == lane) {
                          unsafe2go_ahead = unsafe2go_ahead || 
                            (check_car_s - car_s > 0 && check_car_s - car_s < SAFETY_GAP_FRONT);
                        } else if (current_car_lane == lane + 1) {
                          unsafe2turn_right = unsafe2turn_right || 
                            (check_car_s - car_s > -SAFETY_GAP_BEHIND && check_car_s - car_s < SAFETY_GAP_FRONT);
                        }
                      }

                      double vel_diff = 0; // velocity difference
                      
                      // decide what do to next if it's unsafe to go ahead (car is in the way)
                      if (unsafe2go_ahead) {
                        // attempt to change the lane
                        if(!unsafe2turn_left && lane != 0) {
                          lane--;
                          
                        } else if (!unsafe2turn_right && lane != MAX_HIGHWAY_LANES - 1){
                          lane++;
                          
                        } else {  // change is unsafe - adapt speed of the car infront
                         
                          if (ref_vel > speed_ahead) {
                            ref_vel -= MAX_ACCELERATION;
                          } else {
                            ref_vel = speed_ahead;
                          }
                        }
                      } else {
                        if (ref_vel < MAX_SPEED) {
                          vel_diff += MAX_ACCELERATION;
                        }
                      }

                      // initialize vectors waypoints
                      vector<double> ptsx;
                      vector<double> ptsy;
                      
                      // get referenz points (= car position)
                      double ref_x = car_x;
                      double ref_y = car_y;
                      double ref_yaw = deg2rad(car_yaw);

                      // 
                      if (prev_size < 2) {
                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);

                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(car_x);

                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(car_y);
                      } else {
                        ref_x = previous_path_x[prev_size - 1];
                        ref_y = previous_path_y[prev_size - 1];

                        double ref_x_prev = previous_path_x[prev_size - 2];
                        double ref_y_prev = previous_path_y[prev_size - 2];
                        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);

                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);
                      }

                      // calculate next waypoints
                      vector<double> next_wp0 = getXY(car_s + 30, (2 + LANE_WIDTH * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                      vector<double> next_wp1 = getXY(car_s + 60, (2 + LANE_WIDTH * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                      vector<double> next_wp2 = getXY(car_s + 90, (2 + LANE_WIDTH * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

                      ptsx.push_back(next_wp0[0]);
                      ptsx.push_back(next_wp1[0]);
                      ptsx.push_back(next_wp2[0]);
                      ptsy.push_back(next_wp0[1]);
                      ptsy.push_back(next_wp1[1]);
                      ptsy.push_back(next_wp2[1]);

                      for (int i = 0; i < ptsx.size(); i++) {
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;

                        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
                        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
                      }

                      tk::spline s;
                      s.set_points(ptsx,ptsy);
                      
                      vector<double> next_x_vals;
                      vector<double> next_y_vals;

                      for (int i = 0; i < previous_path_x.size(); i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                      }

                      double target_x = 30.0;
                      double target_y = s(target_x);
                      double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
                      double x_add_on = 0;
                      double timestep = 0.02; // length of timestep [s]
                      double num_points_total = 50;
                      
                      // calculate the next waypoints 
                      for (int i = 1; i <= num_points_total - previous_path_x.size(); i++) {

                        ref_vel += vel_diff;
                        if ( ref_vel > MAX_SPEED ) {
                          ref_vel = MAX_SPEED;
                        } else if ( ref_vel < MAX_ACCELERATION ) {
                          ref_vel = MAX_ACCELERATION;
                        }

                        double N = (target_dist / (timestep * ref_vel / 2.24));
                        double x_point = x_add_on + (target_x) / N;
                        double y_point = s(x_point);
                        
                        x_add_on = x_point;

                        double x_ref = x_point;
                        double y_ref = y_point;

                        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
                        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
                        
                        // reference points to car coordinate system
                        x_point += ref_x;
                        y_point += ref_y;

                        // store next waypoints in vector
                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);
                      }

                      // pass next waypoints
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