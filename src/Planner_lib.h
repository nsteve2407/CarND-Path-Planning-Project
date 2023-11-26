#include <vector>
#include <cmath>
#include "helpers.h"
#include "glog/logging.h"

using namespace std;
struct nbr_vehicle
{
    int id;
    double x;
    double y;
    double vx;
    double vy;
    double d;
    double s;
};

struct vehicle_state
{
    double x;
    double y;
    double yaw;
    double speed;
    double d;
    double s;
};

struct config
{
    float prediction_horizon; //secs
    float time_resolution; //secs
    float safety_window = 15.0; //meters in Frenet (s, i.e lenght along the path)

};

struct point
{
    float x; //m
    float y; //m
};

enum action {KLSU,KLSD,LCL,LCR};

class Prediction
{
    public:
    Prediction();
    bool predict_one_step_constant_velocity(const nbr_vehicle& start_state, nbr_vehicle& next_state,const config& settings);
    vector<nbr_vehicle> predict_single_vehicle_states(const nbr_vehicle& current_state, config& settings);
    vector<nbr_vehicle> predict_nbr_final_states(const vector<nbr_vehicle>& current_state, config& settings);
    vector<vector<nbr_vehicle>> predict_nbr_trajectories(const vector<nbr_vehicle>& current_state, config& settings);

    vector<nbr_vehicle> nbr_vehicle_states_;    

};


class BehaviourPlanner
{
    public:
    BehaviourPlanner();
    int match_vehicle_to_lane(const nbr_vehicle& vehicle_state);
    int match_vehicle_to_lane(const vehicle_state& ego_state);

    bool in_safety_window(const vehicle_state& localization,const nbr_vehicle& nbr_vehicle_state,const config& settings);
    action next_action(const vehicle_state& localization,const vector<nbr_vehicle>& nbrs_last_state,const config& settings);
    action next_action(const vehicle_state& localization,const vector<vector<nbr_vehicle>>& nbrs_trajectories,const config& settings);
    double current_velocity_;
    vector<double> map_waypoints_x_;
    vector<double> map_waypoints_y_;

};

class MotionPlanner
{
    MotionPlanner();
    vector<point> generate_motion_plan(const vehicle_state& localization,const vector<point>& prev_plan,const action& a, const vector<point>& waypoints);

};