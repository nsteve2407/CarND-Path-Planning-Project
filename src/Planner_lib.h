#include "glog/logging.h"
#include "helpers.h"
#include <bits/stdc++.h>
#include <cmath>
#include <unordered_map>
#include <vector>

using namespace std;
// typedef pair<int, State> node;

class node {
public:
  node(float wt, State s) : wt_(wt), state_(s){};
  float wt_;
  State state_;
};

struct CompareNode {
  bool operator()(node const &p1, node const &p2) {
    // return "true" if "p1" is ordered
    // before "p2", for example:
    return p1.wt_ < p2.wt_;
  }
};

struct nbr_vehicle {
  int id;
  double x;
  double y;
  double vx;
  double vy;
  double d;
  double s;
};

struct vehicle_state {
  double x;
  double y;
  double yaw;
  double speed;
  double d;
  double s;
};

struct config {
  float prediction_horizon;   // secs
  float time_resolution;      // secs
  float safety_window = 15.0; // meters in Frenet (s, i.e lenght along the path)
};

struct point {
  float x; // m
  float y; // m
  point(const float &x_, const float &y_) : x(x_), y(y_){};
};

enum action { KLSU, KLSD, LCL, LCR };

class Prediction {
public:
  Prediction();
  bool predict_one_step_constant_velocity(const nbr_vehicle &start_state,
                                          nbr_vehicle &next_state,
                                          const config &settings);
  vector<nbr_vehicle>
  predict_single_vehicle_states(const nbr_vehicle &current_state,
                                config &settings);
  vector<nbr_vehicle>
  predict_nbr_final_states(const vector<nbr_vehicle> &current_state,
                           config &settings);
  vector<vector<nbr_vehicle>>
  predict_nbr_trajectories(const vector<nbr_vehicle> &current_state,
                           config &settings);

  vector<nbr_vehicle> nbr_vehicle_states_;
};

struct State {
  float x;
  float y;
  float theta; // radians
  float v;
  int prev_action; // What action was taken to reach this state:
                   // {0,1,2}{left,straight,right}
  float min_obst_dist = 100000.0;
  State(float x_, float y_, float theta_, float v_, int action_)
      : x(x_), y(y_), theta(theta_), v(v_), prev_action(action_){};
};

double euclideanDistance(const State &pose1, const vector<double> pose2) {
  // Calculate the differences in x, y, and theta
  double dx = pose1.x - pose2[0];
  double dy = pose1.y - pose2[1];

  // If theta represents angles in radians, you might need to handle the
  // periodic nature of angles
  double dtheta = std::fmod(pose1.theta - pose2[2] + M_PI, 2 * M_PI) - M_PI;

  // Calculate the Euclidean distance
  return std::sqrt(dx * dx + dy * dy + dtheta * dtheta);
}

class BehaviourPlanner {
public:
  BehaviourPlanner();
  BehaviourPlanner(vector<double> map_x, vector<double> map_y)
      : map_waypoints_x_(map_x), map_waypoints_y_(map_y){};
  int match_vehicle_to_lane(const nbr_vehicle &vehicle_state);
  int match_vehicle_to_lane(const vehicle_state &ego_state);

  bool in_safety_window(const vehicle_state &localization,
                        const nbr_vehicle &nbr_vehicle_state,
                        const config &settings);
  action next_action(const vehicle_state &localization,
                     const vector<nbr_vehicle> &nbrs_last_state,
                     const config &settings);
  action next_action(const vehicle_state &localization,
                     const vector<vector<nbr_vehicle>> &nbrs_trajectories,
                     const config &settings);
  double current_velocity_;
  vector<double> map_waypoints_x_;
  vector<double> map_waypoints_y_;
  int currnet_vehicle_lane;
};

// Motion PLanner that uses Hybrid A* + Dubins Path
class MotionPlanner : BehaviourPlanner {
  MotionPlanner(vector<double> map_x, vector<double> map_y,
                vector<double> map_s)
      : BehaviourPlanner(map_x, map_y), map_waypoints_s_(map_s){};
  vector<double> map_waypoints_s_;

public:
  vector<point> generate_motion_plan(const vehicle_state &localization,
                                     const vector<point> &prev_plan,
                                     const action &a);

  string state2string_round(const State &s);
  string state2string_no_round(const State &s);
  void string2state(const string &s, float &x, float &y);

  // private:
  float res_x;
  float res_y;
  float res_theta;
  float turn_radius = 8.0;
  float action_change_cost_ = 4.0;
  float obstacle_distacne_cost_wt_ = 10.0;
  float h_cost_wt_ = 6.0;
  float dt_ = 0.02;

  unordered_map<string, float> seen_;
  unordered_map<string, string> optimal_path;

  vector<nbr_vehicle> obstacles;

  int get_lane(const vehicle_state &loc);

  // Unordered map to store visited nodes
  unordered_map<string, float>
      visited_map; // String here is discretized version of state "x,y,teta"
  // Unordered map to store parent nodes and retrieve optimal path
  unordered_map<string, string> optimal_parent;
  State dubins_trasition_function(const State &start, float v_cmd, int &action,
                                  float &radius);

  bool check_valid(State &State, vector<float> &limits);
  bool reached_goal(const State &s, const State &goal);
  float cost(const State &start, const State &end, vector<double> &goal);
  bool check_visited(const State &s);
  vector<point> generate_traj(const unordered_map<string, string> &p,
                              const string &start, const string &end);
};
}
;