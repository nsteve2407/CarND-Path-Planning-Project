#include <vector>

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
    Prediction();
    vector<nbr_vehicle> predict_end_state(const vector<nbr_vehicle>& current_state, config& settings);
    vector<vector<nbr_vehicle>> predict_nbr_trajectory(const vector<nbr_vehicle>& current_state, config& settings);

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