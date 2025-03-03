#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Planner_lib.h"
#include "glog/logging.h"
#include "json.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <uWS/uWS.h>
#include <vector>

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

  // Init  planning layers
  Prediction predict;
  BehaviourPlanner behavior(map_waypoints_x, map_waypoints_y);

  config settings;
  settings.time_resolution = 0.02;
  settings.prediction_horizon = 0.5;

  // Variable initialization
  vector<nbr_vehicle> nbr_current_state;

  MotionPlanner motion_planner(map_waypoints_x, map_waypoints_y,
                               map_waypoints_s, nbr_current_state);

  // std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &nbr_current_state,
               &predict, &behavior, &motion_planner,
               &settings](uWS::WebSocket<uWS::SERVER> ws, char *data,
                          size_t length, uWS::OpCode opCode) {
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

          vehicle_state ego_state(car_x, car_y, car_yaw, car_speed, car_d,
                                  car_s);

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          vector<vector<double>> prev_path(2);
          prev_path.push_back(previous_path_x);
          prev_path.push_back(previous_path_y);

          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          LOG(INFO) << "Sensor Fusion Data";
          for (auto car : sensor_fusion) {
            // LOG(INFO) << "Car ID: " << car[0];
            // LOG(INFO) << "Car X: " << car[1];
            // LOG(INFO) << "Car Y: " << car[2];
            // LOG(INFO) << "Car Vx: " << car[3];
            // LOG(INFO) << "Car Vy: " << car[4];
            // LOG(INFO) << "Car s: " << car[5];
            // LOG(INFO) << "Car d: " << car[6];
            nbr_current_state.push_back(nbr_vehicle(
                car[0], car[1], car[2], car[3], car[4], car[6], car[5]));
          }

          // auto nbr_predictions =
          //     predict.predict_nbr_final_states(nbr_current_state, settings);
          // LOG(INFO) << "Predictions generated: " << nbr_predictions.size();
          // if (nbr_predictions.size() > 0) {
          //   LOG(INFO) << "Dummy prediction x: " << nbr_predictions[0].x;
          //   LOG(INFO) << "Dummy prediction y: " << nbr_predictions[0].y;
          //   LOG(INFO) << "Dummy prediction ID: " << nbr_predictions[0].id;
          //   LOG(INFO) << "Dummy prediction x: "
          //             << nbr_predictions[nbr_predictions.size() - 1].x;
          //   LOG(INFO) << "Dummy prediction y: "
          //             << nbr_predictions[nbr_predictions.size() - 1].y;
          //   LOG(INFO) << "Dummy prediction ID: "
          //             << nbr_predictions[nbr_predictions.size() - 1].id;
          // }

          action a =
              behavior.next_action(ego_state, nbr_current_state, settings);
          LOG(INFO) << "Behavior plan generated: " << (int)a;

          motion_planner.obstacles = nbr_current_state;
          vector<vector<double>> path =
              motion_planner.generate_motion_plan(ego_state, prev_path, a);

          LOG(INFO) << "Motion plan generated points: " << path[0].size();

          next_x_vals = path[0];
          next_y_vals = path[1];

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          // exit(-1);
        } // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
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