#include "Planner_lib.h"

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
    nbr_vehicle next_state = start_state;
    // Constat Velocity propogation
    next_state.x = next_state.x + next_state.vx * settings.time_resolution;
    next_state.y = next_state.y + next_state.vy * settings.time_resolution;

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

  for (int i = 0; i < state_predictions.size(); i++) {
    success = predict_one_step_constant_velocity(state, next_state, settings);
    state_predictions[0] = next_state;
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
  if (vehicle_state.d < 0.0 || vehicle_state.d > 12.0) {
    return -1; // Oncoming traffic
  }
  // Lanes are 4m wide
  else if (vehicle_state.d > 0.0 && vehicle_state.d < 4.0) {
    return 0; // Left lane
  } else if (vehicle_state.d > 4.0 && vehicle_state.d < 8.0) {
    return 0; // Middle lane traffic
  } else if (vehicle_state.d > 8.0 && vehicle_state.d < 12.0) {
    return 0; // Right lane traffic
  }
}

int BehaviourPlanner::match_vehicle_to_lane(
    const vehicle_state &vehicle_state) {
  if (vehicle_state.d < 0.0 || vehicle_state.d > 12.0) {
    return -1; // Oncoming traffic
  }
  // Lanes are 4m wide
  else if (vehicle_state.d > 0.0 && vehicle_state.d < 4.0) {
    return 0; // Left lane
  } else if (vehicle_state.d > 4.0 && vehicle_state.d < 8.0) {
    return 0; // Middle lane traffic
  } else if (vehicle_state.d > 8.0 && vehicle_state.d < 12.0) {
    return 0; // Right lane traffic
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
  vector<bool> lane_blocked_status = {false, false, false};

  for (auto vehicle : nbrs_last_state) {
    int lane = match_vehicle_to_lane(vehicle);
    if (!lane_blocked_status[lane]) {
      if (in_safety_window(localization, vehicle, settings)) {
        lane_blocked_status[lane] = true;
      }
    }
  }
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

    // If Left lane check center
    if (currnet_vehicle_lane == 0) {
      if (!lane_blocked_status[1]) {
        return LCR;
      } else {
        return KLSD; // Keep Lane Slow down
      }
    }
    // If right lane check center
    if (currnet_vehicle_lane == 2) {
      if (!lane_blocked_status[1]) {
        return LCL;
      } else {
        return KLSD; // Keep Lane Slow down
      }
    }
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

  if (a == LCL) {
    lane -= 1;
  } else if (a == LCR) {
    lane += 1;
  } else if (a == KLSU) {
    delta_v = 0.224;
  } else {
    delta_v = -0.224;
  }

  // goal as (x,y,theta)
  vector<double> goal_state =
      getXY(localization.s + goal_lookup_distance, 2 + 4 * lane,
            map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);

  float min_x = min<float>(float(goal_state[0]), localization.x);
  float max_x = max<float>(float(goal_state[0]), localization.x);
  float min_y = min<float>(float(goal_state[1]), localization.y);
  float max_y = max<float>(float(goal_state[1]), localization.y);

  vector<float> limits = {min_x, max_x, min_y, max_y};

  float v;

  priority_queue<node, vector<node>, CompareNode> pq;

  State start_state =
      State(localization.x, localization.y, localization.yaw, v, -1);

  State goal_state_f =
      State(goal_state[0], goal_state[1], goal_state[2], v, -1);
  string goal_state_id = state2string_round(goal_state_f);

  pq.push(node(0.0, start_state));
  string start_state_id = state2string_round(start_state);

  State dest(0.0, 0.0, 0.0, 0.0, -1);
  string node_id, node_id_start;
  string optimal_child_node_id;
  while (pq.size() > 0) {
    v = min<float>(localization.speed + delta_v, 50.0 * 0.447);
    v = max<float>(localization.speed + delta_v, 0.447);

    float node_cost = 1000000.0;
    float node_min_cost = 1000000.0;
    auto n = pq.top();
    pq.pop();
    node_id_start = state2string_no_round(n.state_);

    for (int action = 0; action < 3; action++) {
      dest = dubins_trasition_function(n.state_, v, action, turn_radius);
      dest.prev_action = action;
      // Convert to string
      node_id = state2string_round(dest);
      // Check if valid
      if (check_valid(dest, limits)) {

        // Check if reached goal
        if (reached_goal(dest, goal_state_f)) {
          optimal_path[node_id_start] = state2string_no_round(dest);
          return generate_traj(optimal_path, start_state_id, goal_state_id);
        }

        // Caculate cost
        node_cost = n.wt_ + cost(n.state_, dest, goal_state);

        // Check if visited
        if (seen_.find(node_id) == seen_.end()) {
          // Not see before so add
          seen_[node_id] = node_cost;
          pq.push(node(node_cost, dest));
          // If visited compare costs and adjust cost and parent to min
        } else {
          if (node_cost < seen_[node_id]) {
            seen_[node_id] = node_cost;
            // Add to pq if fist time or if modified update the pq value?
            pq.push(node(node_cost, dest));
          }
        }
        if (node_cost < node_min_cost) {
          node_min_cost = node_cost;
          optimal_child_node_id = state2string_no_round(dest);
        }
      }
    }
    optimal_path[node_id_start] = optimal_child_node_id;
  }
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

bool MotionPlanner::check_valid(State &State, vector<float> &limits) {
  if (State.x > limits[0] && State.x < limits[1] && State.y > limits[2] &&
      State.y < limits[3]) {
    for (auto car : obstacles) {
      State.min_obst_dist = min<float>(
          State.min_obst_dist, distance(State.x, State.y, car.x, car.y));
    }
    if (State.min_obst_dist < 3.0) {
      return true;
    }
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

  while (node_id != end) {
    next_node = optimal_path[node_id];
    string2state(next_node, pt.x, pt.y);
    p_x.push_back(double(x));
    p_y.push_back(double(y));

    node_id = next_node;
  }

  return {p_x, p_y};
}

// Rounding based on resolution
string MotionPlanner::state2string_round(const State &s) {
  float x = round(s.x / res_x) * res_x;
  float y = round(s.y / res_y) * res_y;
  float theta = round(s.theta / res_theta) * res_theta;

  return to_string(x) + "," + to_string(y) + "," + to_string(theta);
}

string state2string_no_round(const State &s) {
  return to_string(round_to<float>(s.x, 0.01)) + "," +
         to_string(round_to<float>(s.y, 0.01)) + "," +
         to_string(round_to<float>(s.theta, 0.01)) + ",";
}

void string2state(const string &s, float &x, float &y) {
  stringstream ss(s);

  // Temporary string to hold each token
  std::string word;

  // Use getline to split the string at commas
  getline(ss, word, ',');
  x = stof(word);
  getline(ss, word, ',');
  y = stof(word);
}