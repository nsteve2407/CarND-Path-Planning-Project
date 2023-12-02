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

vector<point>
MotionPlanner::generate_motion_plan(const vehicle_state &localization,
                                    const vector<point> &prev_plan,
                                    const action &a) {
  // Get goal point from localization info, waypoints and action
  int lane = match_vehicle_to_lane(localization);
  float delta_v = 0.0;
  float goal_lookup_distance = 30.0;
  float turn_radius = 8.0;

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

  float v = localization.speed;

  priority_queue<node, vector<node>, CompareNode> pq;

  pq.push(node(0.0,
               State(localization.x, localization.y, localization.yaw, v, -1)));

  while (pq.size() > 0) {
    auto n = pq.top();
    pq.pop();

    for (int action = 0; action < 3; action++) {
      auto dest = dubins_trasition_function(n.state_, v, action, turn_radius);

      // Check if valid

      // Check if reached goal`

      // Caculate cost

      // Check if visited
      // Convert to string
      // If visited compare costs and adjust cost and parent to min

      //  IF not visited add it

      // Add to pq if fist time or if modified update the pq value?
    }
  }
}
