#include "glog/logging.h"
#include "helpers.h"
#include <algorithm>
#include <bits/stdc++.h>
#include <cmath>
#include <iomanip> // Include the header for setprecision
#include <sstream>
#include <unordered_map>
#include <vector>

using namespace std;
// typedef pair<int, State> node;
template <typename T>

T round_to(T value, T precision = 1.0) {
  return std::round(value / precision) * precision;
}

struct State {
  float x;
  float y;
  float theta; // radians
  float v;
  int prev_action; // What action was taken to reach this state:
                   // {0,1,2}{left,straight,right}
  float min_obst_dist = 7000.0;
  State(){};
  State(float x_, float y_, float theta_, float v_, int action_)
      : x(x_), y(y_), theta(theta_), v(v_), prev_action(action_){};
};

class node {
public:
  // node(float wt, State s, node &p) : wt_(wt), state_(s), parent(&p){};
  node() : parent(nullptr){};
  float wt_;
  State state_;
  shared_ptr<node> parent;
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
  nbr_vehicle(int num = 0, double x_ = 0.0, double y_ = 0.0, double vx_ = 0.0,
              double vy_ = 0.0, double d_ = 0.0, double s_ = 0.0)
      : id(num), x(x_), y(y_), vx(vx_), vy(vy_), d(d_), s(s_){};
};

struct vehicle_state {
  double x;
  double y;
  double yaw;
  double speed;
  double d;
  double s;
  vehicle_state(double x_, double y_, double yaw_, double speed_, double d_,
                double s_)
      : x(x_), y(y_), yaw(yaw_), speed(speed_), d(d_), s(s_){};
};

struct config {
  float prediction_horizon;     // secs
  float time_resolution = 0.02; // secs
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
  Prediction(){};
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

public:
  MotionPlanner(vector<double> map_x, vector<double> map_y,
                vector<double> map_s, vector<nbr_vehicle> nbrs)
      : BehaviourPlanner(map_x, map_y), map_waypoints_s_(map_s),
        obstacles(nbrs){};
  vector<double> map_waypoints_s_;
  vector<vector<double>>
  generate_motion_plan(const vehicle_state &localization,
                       const vector<vector<double>> &prev_plan,
                       const action &a);

  string state2string_round(const State &s);
  string state2string_no_round(const State &s);
  void string2state(const string &s, float &x, float &y);

  // private:
  float res_x = 0.1;
  float res_y = 0.1;
  float res_theta = 5 * (3.14f / 180.0f);
  float turn_radius = 10.0;
  float action_change_cost_ = 4.0;
  float obstacle_distacne_cost_wt_ = 6.0;
  float h_cost_wt_ = 20.0;
  float dt_ = 0.04;

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
  vector<vector<double>> generate_traj(const unordered_map<string, string> &p,
                                       const string &start, const string &end);
  vector<vector<double>> generate_traj(shared_ptr<node> &goal_node,
                                       vector<shared_ptr<node>> &nodes);
};

// Addinng function definitions here (not best practice)

// struct nbr_vehicle
// {
//     int id;
//     double x;
//     double y;
//     double vx;
//     double vy;
//     double d;
//     double s;
// };

bool Prediction::predict_one_step_constant_velocity(
    const nbr_vehicle &start_state, nbr_vehicle &next_state,
    const config &settings) {
  try {
    next_state.id = start_state.id;

    // Constat Velocity propogation
    next_state.x = start_state.x + start_state.vx * settings.time_resolution;
    next_state.y = start_state.y + start_state.vy * settings.time_resolution;
    next_state.vx = start_state.vx;
    next_state.vx = start_state.vy;

    LOG(INFO) << "(step) Input state vx=" << start_state.vx;
    LOG(INFO) << "(step) Input state vy=" << start_state.vy;
    LOG(INFO) << "(step) Calculated ns id=" << next_state.id;
    LOG(INFO) << "(step) Calculated ns x=" << next_state.x;
    LOG(INFO) << "(step) Calculated ns y=" << next_state.y;
    LOG(INFO) << "(step) Calculated ns vx=" << next_state.vx;
    LOG(INFO) << "(step) Calculated ns vy=" << next_state.vy;

    // To do : Frenet update

    return true;
  } catch (std::exception e) {
    LOG(ERROR) << "Error in single step prediciton";
    return false;
  }
}

vector<nbr_vehicle>
Prediction::predict_single_vehicle_states(const nbr_vehicle &current_state,
                                          config &settings) {
  // Constant Velocity Motion Model
  vector<nbr_vehicle> state_predictions(settings.prediction_horizon /
                                        settings.time_resolution);

  nbr_vehicle state = current_state;
  nbr_vehicle next_state;
  bool success;
  LOG(INFO) << "First iteration ! aka source state:" << state.id;

  for (int i = 0; i < state_predictions.size(); i++) {
    success = predict_one_step_constant_velocity(state, next_state, settings);
    LOG(INFO) << "State ID:" << state.id;
    LOG(INFO) << "State x:" << state.x;
    LOG(INFO) << "State y:" << state.y;
    LOG(INFO) << "Next-State ID:" << next_state.id;
    LOG(INFO) << "Next-State x:" << next_state.x;
    LOG(INFO) << "Next-State y:" << next_state.y;
    state_predictions[i] = next_state;
    state = next_state;
  }

  return state_predictions;
}

vector<vector<nbr_vehicle>>
Prediction::predict_nbr_trajectories(const vector<nbr_vehicle> &current_state,
                                     config &settings) {
  // One trajectory for each vehicle in current state
  vector<vector<nbr_vehicle>> trajectories(current_state.size());

  // Iterate over all vehicles
  for (int i = 0; i < current_state.size(); i++) {
    trajectories[i] = predict_single_vehicle_states(current_state[i], settings);
  }

  return trajectories;
}

// Variant of predict_nbr_trajectories, that retruns only the state at the end
// of the time window
vector<nbr_vehicle>
Prediction::predict_nbr_final_states(const vector<nbr_vehicle> &current_state,
                                     config &settings) {
  vector<vector<nbr_vehicle>> trajectories =
      predict_nbr_trajectories(current_state, settings);
  vector<nbr_vehicle> end_states(current_state.size());

  for (int i = 0; i < trajectories.size(); i++) {
    end_states[i] = trajectories[i][trajectories[i].size() - 1];
  }

  return end_states;
}

int BehaviourPlanner::match_vehicle_to_lane(const nbr_vehicle &vehicle_state) {
  LOG(INFO) << "Vehice d:" << vehicle_state.d;

  if (vehicle_state.d < 0.0 || vehicle_state.d > 12.0) {
    return -1; // Oncoming traffic
  }
  // Lanes are 4m wide
  else if (vehicle_state.d > 0.0 && vehicle_state.d < 4.0) {
    return 0; // Left lane
  } else if (vehicle_state.d > 4.0 && vehicle_state.d < 8.0) {
    return 1; // Middle lane traffic
  } else if (vehicle_state.d > 8.0 && vehicle_state.d < 12.0) {
    return 2; // Right lane traffic
  }
}

int BehaviourPlanner::match_vehicle_to_lane(
    const vehicle_state &vehicle_state) {
  LOG(INFO) << "Vehice d:" << vehicle_state.d;

  if (vehicle_state.d < 0.0 || vehicle_state.d > 12.0) {
    return -1; // Oncoming traffic
  }
  // Lanes are 4m wide
  else if (vehicle_state.d > 0.0 && vehicle_state.d < 4.0) {
    return 0; // Left lane
  } else if (vehicle_state.d > 4.0 && vehicle_state.d < 8.0) {
    return 1; // Middle lane traffic
  } else if (vehicle_state.d > 8.0 && vehicle_state.d < 12.0) {
    return 2; // Right lane traffic
  }
}

bool BehaviourPlanner::in_safety_window(const vehicle_state &localization,
                                        const nbr_vehicle &nbr_vehicle_state,
                                        const config &settings) {
  if (nbr_vehicle_state.s < localization.s + settings.safety_window) {
    return true;
  }

  else if (nbr_vehicle_state.s > localization.s - settings.safety_window) {
    return true;
  }

  return false;
}

action BehaviourPlanner::next_action(const vehicle_state &localization,
                                     const vector<nbr_vehicle> &nbrs_last_state,
                                     const config &settings) {
  currnet_vehicle_lane = match_vehicle_to_lane(localization);
  LOG(INFO) << "Matched ego vehicle to lane:" << currnet_vehicle_lane;
  vector<bool> lane_blocked_status = {false, false, false};

  for (auto vehicle : nbrs_last_state) {
    int lane = match_vehicle_to_lane(vehicle);
    LOG(INFO) << "Matched NBR vehicle to lane:" << lane;

    if (lane != -1) {
      if (!lane_blocked_status[lane]) {
        if (in_safety_window(localization, vehicle, settings)) {
          lane_blocked_status[lane] = true;
        }
      }
    }
  }
  LOG(INFO) << "Lane matching done";

  // If current lane not blocked
  if (!lane_blocked_status[currnet_vehicle_lane]) {
    return KLSU; // Keep Lane Speed Up
  }
  // If current lane is blocked
  if (lane_blocked_status[currnet_vehicle_lane]) {
    // If center lane check left and right
    if (currnet_vehicle_lane == 1) {
      if (!lane_blocked_status[0]) {
        return LCL;
      } else if (!lane_blocked_status[2]) {
        return LCR;
      } else {
        return KLSD; // Keep Lane Slow down
      }
    }

    LOG(INFO) << "Current lane check done";

    // If Left lane check center
    if (currnet_vehicle_lane == 0) {
      if (!lane_blocked_status[1]) {
        return LCR;
      } else {
        return KLSD; // Keep Lane Slow down
      }
    }
    LOG(INFO) << "Left lane check done";

    // If right lane check center
    if (currnet_vehicle_lane == 2) {
      if (!lane_blocked_status[1]) {
        return LCL;
      } else {
        return KLSD; // Keep Lane Slow down
      }
    }
    LOG(INFO) << "Right lane check done";
  }
}

vector<vector<double>>
MotionPlanner::generate_motion_plan(const vehicle_state &localization,
                                    const vector<vector<double>> &prev_plan,
                                    const action &a) {
  // Get goal point from localization info, waypoints and action
  int lane = match_vehicle_to_lane(localization);
  float delta_v = 0.0;
  float goal_lookup_distance = 10.0;

  LOG(INFO) << "(MPL) Ego lane:" << lane;

  if (a == LCL) {
    lane -= 1;
  } else if (a == LCR) {
    lane += 1;
  } else if (a == KLSU) {
    delta_v = 0.224;
  } else {
    delta_v = -0.224;
  }

  LOG(INFO) << "(MPL) Delta v:" << delta_v;

  // goal as (x,y,theta)
  vector<double> goal_state =
      getXY(localization.s + goal_lookup_distance, 2 + 4 * lane,
            map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);

  float min_x = min<float>(float(goal_state[0]), localization.x);
  float max_x = max<float>(float(goal_state[0]), localization.x);
  float min_y = min<float>(float(goal_state[1]), localization.y);
  float max_y = max<float>(float(goal_state[1]), localization.y);

  vector<float> limits = {min_x - 1.0f, max_x + 1.0f, min_y - 1.0f,
                          max_y + 1.0f};

  float v = min<float>(localization.speed + delta_v, 50.0 * 0.277);
  v = max<float>(localization.speed + delta_v, 0.277);

  priority_queue<node, vector<node>, CompareNode> pq;

  State start_state =
      State(localization.x, localization.y, localization.yaw, v, -1);

  LOG(INFO) << "(MPL) Start state :" << start_state.x << ", " << start_state.y
            << ", " << start_state.theta;

  State goal_state_f =
      State(goal_state[0], goal_state[1], start_state.theta, v, -1);
  LOG(INFO) << "(MPL) Goal state :" << goal_state_f.x << ", " << goal_state_f.y
            << ", " << goal_state_f.theta;

  string goal_state_id = state2string_no_round(goal_state_f);

  LOG(INFO) << "(MPL) Goal state id: " << goal_state_id;

  shared_ptr<node> start(new node);
  start->state_ = start_state;
  start->parent = nullptr;
  start->wt_ = 0.0;

  vector<shared_ptr<node>> nodes;
  nodes.push_back(start);

  pq.push(*start);
  string start_state_id = state2string_no_round(start_state);
  LOG(INFO) << "(MPL) Start state id: " << start_state_id;

  float node_cost = 1000000.0;
  float node_min_cost = 1000000.0;

  State dest(0.0, 0.0, 0.0, 0.0, -1);
  string node_id, node_id_start;
  string optimal_child_node_id;
  while (pq.size() > 0) {
    float node_cost = 1000000.0;
    float node_min_cost = 1000000.0;

    const node &current_node_ref = pq.top();

    // Create a shared_ptr and copy the element
    shared_ptr<node> current_node = make_shared<node>(current_node_ref);

    pq.pop();
    node_id_start = state2string_no_round(current_node->state_);
    LOG(INFO) << "(MPL) Popped node id: " << node_id_start;
    LOG(INFO) << "(MPL) Velocity v: " << v;

    for (int action = 0; action < 3; action++) {
      dest = dubins_trasition_function(current_node->state_, v, action,
                                       turn_radius);
      LOG(INFO) << "(MPL) Take action:" << action;
      LOG(INFO) << "(MPL) Dest node (dubins):" << dest.x << ", " << dest.y
                << ", " << dest.theta;

      dest.prev_action = action;
      // Convert to string
      node_id = state2string_round(dest);
      LOG(INFO) << "(MPL) Dest node id: " << node_id;

      // Check if valid
      LOG(INFO) << "Limits: x_min,x_max" << limits[0] << ", " << limits[1];
      LOG(INFO) << "Limits: y_min,y_max" << limits[2] << ", " << limits[3];
      auto valid = check_valid(dest, limits);
      LOG(INFO) << "State is valid? :" << valid;
      if (valid) {
        LOG(INFO) << "(MPL) Node id " << node_id << "is valid: ";

        // Check if reached goal
        if (reached_goal(dest, goal_state_f)) {
          LOG(INFO) << "(MPL) Reached goal state";

          optimal_path[node_id_start] = state2string_no_round(dest);
          // return generate_traj(optimal_path, start_state_id,
          //                      state2string_no_round(dest));
          shared_ptr<node> child(new node);
          // shared_ptr<node> parent_pt(&n);
          // child->parent = parent_pt;
          child->parent = current_node;
          child->wt_ = 0.0;
          child->state_ = dest;
          nodes.push_back(child);
          // node final_node(0.0, dest, &n);
          LOG(INFO) << "(MPL) Start trajectory generation";
          return generate_traj(child, nodes);
        }

        // Caculate cost
        node_cost =
            current_node->wt_ + cost(current_node->state_, dest, goal_state);

        LOG(INFO) << "(MPL) Node cost: " << node_cost;

        // Check if visited
        if (seen_.find(node_id) == seen_.end()) {
          LOG(INFO) << "(MPL) Node id " << node_id << " not seen before";

          // Not see before so add
          seen_[node_id] = node_cost;
          shared_ptr<node> child(new node);
          // shared_ptr<node> parent_pt(&n);
          // child->parent = parent_pt;
          child->parent = current_node;
          child->wt_ = node_cost;
          child->state_ = dest;
          pq.push(*child);
          nodes.push_back(child);
          // If visited compare costs and adjust cost and parent to min
        } else {
          LOG(INFO) << "(MPL) Node id " << node_id << " has been seen before";
          if (node_cost < seen_[node_id]) {
            LOG(INFO) << "(MPL) Node id " << node_id
                      << " current cost: " << node_cost
                      << " Previous cost: " << seen_[node_id];

            seen_[node_id] = node_cost;
            shared_ptr<node> child(new node);
            // shared_ptr<node> parent_pt(&n);
            // child->parent = &parent_pt;
            // child->parent = parent_pt;
            child->parent = current_node;
            child->wt_ = node_cost;
            child->state_ = dest;
            pq.push(*child);
            nodes.push_back(child);
            // pq.push(node(node_cost, dest, &n));
          }
        }
        if (node_cost < node_min_cost) {
          node_min_cost = node_cost;
          optimal_child_node_id = state2string_no_round(dest);
        }
      }
    }
    LOG(INFO) << "----Adding key node: " << node_id_start;
    LOG(INFO) << "-----Add value:Optimal child node: " << optimal_child_node_id;
    LOG(INFO) << "(MPL) Goal state id: " << goal_state_id;

    optimal_path[node_id_start] = optimal_child_node_id;
    v = min<float>(v + delta_v, 50.0 * 0.277);
    v = max<float>(v + delta_v, 0.277);
  }

  LOG(FATAL) << "(MPL) Did not reach goal state";
  exit(-1);
  // optimal_path[node_id_start] = state2string_round(dest);
  // return generate_traj(optimal_path, start_state_id, goal_state_id);
}

State MotionPlanner::dubins_trasition_function(const State &start, float v,
                                               int &action, float &r) {
  // Calculate the change in position and orientation
  if (action != 1) {
    if (action == 2) {
      r = r * -1.0;
    }
    float deltaTheta = v / r * dt_;
    float deltaX = r * (1 - std::cos(deltaTheta));
    float deltaY = r * std::sin(deltaTheta);

    // Calculate the new position and orientation
    float newX = start.x + deltaX * std::cos(start.theta) -
                 deltaY * std::sin(start.theta);
    float newY = start.y + deltaX * std::sin(start.theta) +
                 deltaY * std::cos(start.theta);
    float newTheta = start.theta + deltaTheta;
    return State(newX, newY, newTheta, v, action);
  }

  if (action == 1) {
    float deltaTheta = 0.0;
    float deltaX = v * cos(start.theta) * dt_;
    float deltaY = v * sin(start.theta) * dt_;
    float newX = start.x + deltaX;
    float newY = start.y + deltaY;
    float newTheta = start.theta + deltaTheta;
    return State(newX, newY, newTheta, v, action);
  }
}

bool MotionPlanner::check_valid(State &s, vector<float> &limits) {
  if (s.x > limits[0] && s.x < limits[1] && s.y > limits[2] &&
      s.y < limits[3]) {

    LOG(INFO) << "State within search bounds";
    LOG(INFO) << "State initial min obj distance:" << s.min_obst_dist;

    for (auto car : obstacles) {
      s.min_obst_dist =
          min<float>(s.min_obst_dist, distance(s.x, s.y, car.x, car.y));
    }
    LOG(INFO) << "State final min obj distance:" << s.min_obst_dist;

    if (s.min_obst_dist < 3.0) {
      return false;
    }
    return true;
  }
  return false;
}

bool MotionPlanner::reached_goal(const State &s, const State &goal) {
  if (round(s.x) == round(goal.x) && round(s.y) == round(goal.y) &&
      round(s.theta * (180 / 3.14)) == round(goal.theta * (180 / 3.14))) {
    return true;
  }
  return false;
}

float MotionPlanner::cost(const State &start, const State &end,
                          vector<double> &goal) {
  float cost_ = 0.0;
  if (start.prev_action != end.prev_action) {
    cost_ += action_change_cost_;
  }
  cost_ = cost_ + (end.min_obst_dist * obstacle_distacne_cost_wt_) +
          h_cost_wt_ * (euclideanDistance(end, goal));
  // Add acceleration(lateral) cost?
  return cost_;
}

vector<vector<double>>
MotionPlanner::generate_traj(const unordered_map<string, string> &p,
                             const string &start, const string &end) {
  point pt(0.0, 0.0);
  vector<double> p_x;
  vector<double> p_y;
  float x, y;
  string node_id = start;
  string next_node;
  string2state(node_id, x, y);
  p_x.push_back(double(x));
  p_y.push_back(double(y));

  // LOG(INFO) << "Current key map:";
  // for (auto x : p) {
  // LOG(INFO) << "Key: " << x.first << " Value: " << x.second;
  // }
  // LOG(INFO) << "Start search for ids";

  while (node_id != end) {
    LOG(INFO) << "Current search id: " << node_id;
    next_node = optimal_path.at(node_id);
    LOG(INFO) << "Next node id: " << next_node;

    string2state(next_node, pt.x, pt.y);
    p_x.push_back(double(x));
    p_y.push_back(double(y));

    node_id = next_node;
  }

  return {p_x, p_y};
}

// vector<vector<double>>
// MotionPlanner::generate_traj(node &goal_node, vector<shared_ptr<node>>
// &nodes) {
//   shared_ptr<node> p(&goal_node);

//   stack<double, vector<double>> stack_x;
//   stack<double, vector<double>> stack_y;

//   while (p) {
//     LOG(INFO) << "(MPL) [Traj_gen], #nodes: " << nodes.size();
//     LOG(INFO) << "(MPL) [Traj_gen], x= " << p->state_.x
//               << " y= " << p->state_.y;
//     stack_x.push(p->state_.x);
//     stack_y.push(p->state_.y);
//     p = p->parent;
//   }

//   vector<double> points_x(stack_x.size());
//   vector<double> points_y(stack_x.size());

//   for (int i = 0; i < points_x.size(); i++) {
//     points_x.push_back(stack_x.top());
//     points_y.push_back(stack_y.top());
//     stack_x.pop();
//     stack_y.pop();
//   }

//   return {points_x, points_y};
// }

vector<vector<double>>
MotionPlanner::generate_traj(shared_ptr<node> &goal_node,
                             vector<shared_ptr<node>> &nodes) {
  vector<double> points_x;
  vector<double> points_y;

  // Start from the goal node
  // shared_ptr<node> current = make_shared<node>(goal_node);

  // const node &current_node_ref = goal_node;

  // Create a shared_ptr and copy the element
  shared_ptr<node> current = goal_node;

  while (current) {
    LOG(INFO) << "(MPL) [Traj_gen], #nodes: " << nodes.size();
    LOG(INFO) << "(MPL) [Traj_gen], x= " << current->state_.x
              << " y= " << current->state_.y;

    points_x.push_back(current->state_.x);
    points_y.push_back(current->state_.y);

    // Move to the parent node
    current = current->parent;
  }

  // Reverse the vectors to get the trajectory from start to goal
  reverse(points_x.begin(), points_x.end());
  reverse(points_y.begin(), points_y.end());

  return {points_x, points_y};
}

// Rounding based on resolution
// string MotionPlanner::state2string_round(const State &s) {
//   float x = round(s.x / res_x) * res_x;
//   float y = round(s.y / res_y) * res_y;
//   LOG(INFO) << "[Conversion] input x:" << s.x;
//   LOG(INFO) << "[Conversion] onput x:" << x;
//   LOG(INFO) << "[Conversion] onput string x:" << to_string(x);
//   LOG(INFO) << "[Conversion] input y:" << s.y;
//   LOG(INFO) << "[Conversion] onput y:" << y;
//   LOG(INFO) << "[Conversion] onput string y:" << to_string(y);

//   float theta = round(s.theta / res_theta) * res_theta;

//   return to_string(x) + "," + to_string(y) + "," + to_string(theta);
// }

string MotionPlanner::state2string_round(const State &s) {
  float x = round(s.x / res_x) * res_x;
  float y = round(s.y / res_y) * res_y;
  // LOG(INFO) << "[Conversion] input x:" << s.x;
  // LOG(INFO) << "[Conversion] output x:" << x;
  // LOG(INFO)
  // << "[Conversion] output string x:" << std::fixed << std::setprecision(2)
  //     << x; // Set precision to 2 decimal places and use fixed-point notation
  // LOG(INFO) << "[Conversion] input y:" << s.y;
  // LOG(INFO) << "[Conversion] output y:" << y;
  // LOG(INFO)
  //     << "[Conversion] output string y:" << std::fixed <<
  //     std::setprecision(2)
  // << y; // Set precision to 2 decimal places and use fixed-point notation

  float theta = round(s.theta / res_theta) * res_theta;

  std::ostringstream oss; // Create a stringstream for concatenating strings
  oss << std::fixed << std::setprecision(2) << x << "," << y << ","
      << theta; // Set precision to 2 decimal places and use fixed-point
                // notation for the entire string

  return oss.str(); // Convert stringstream to string and return
}

string MotionPlanner::state2string_no_round(const State &s) {
  return to_string(round_to<float>(s.x, 0.01)) + "," +
         to_string(round_to<float>(s.y, 0.01)) + "," +
         to_string(round_to<float>(s.theta, 0.01)) + ",";
}

void MotionPlanner::string2state(const string &s, float &x, float &y) {
  stringstream ss(s);

  // Temporary string to hold each token
  std::string word;

  // Use getline to split the string at commas
  getline(ss, word, ',');
  x = stof(word);
  getline(ss, word, ',');
  y = stof(word);
}