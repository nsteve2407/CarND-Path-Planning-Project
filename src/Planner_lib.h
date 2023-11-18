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
    float safety_window; //secs

};

struct point
{
    float x; //m
    float y; //m
};

enum action {KLF,KLS,LCL,LCR};

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
    BehaviourPlanner();
    action next_action(const vehicle_state& localization,const vector<nbr_vehicle>& nbrs_last_state,const config& settings);
    action next_action(const vehicle_state& localization,const vector<vector<nbr_vehicle>>& nbrs_trajectories,const config& settings);

};

class MotionPlanner
{
    MotionPlanner();
    vector<point> generate_motion_plan(const vehicle_state& localization,const vector<point>& prev_plan,const action& a, const vector<point>& waypoints);

};