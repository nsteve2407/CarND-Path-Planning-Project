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

bool Prediction::predict_one_step_constant_velocity(const nbr_vehicle& start_state, nbr_vehicle& next_state,const config& settings)
{
    try
    {
    nbr_vehicle next_state = start_state;
    // Constat Velocity propogation
    next_state.x = next_state.x + next_state.vx*settings.time_resolution;
    next_state.y = next_state.y + next_state.vy*settings.time_resolution;

    // To do : Frenet update


    return true;
    }
    catch(std::exception e)
    {
        LOG(ERROR)<<"Error in single step prediciton";
        return false;
    }
}


vector<nbr_vehicle> Prediction::predict_single_vehicle_states(const nbr_vehicle& current_state, config& settings)
{
    // Constant Velocity Motion Model
    vector<nbr_vehicle> state_predictions(settings.prediction_horizon/settings.time_resolution);

    nbr_vehicle state = current_state;
    nbr_vehicle next_state;
    bool success;

    for(int i=0;i<state_predictions.size();i++)
    {
        success =  predict_one_step_constant_velocity(state,next_state,settings);
        state_predictions[0] = next_state;
        state = next_state;
    }

    return state_predictions;
}



vector<vector<nbr_vehicle>> Prediction::predict_nbr_trajectories(const vector<nbr_vehicle>& current_state, config& settings)
{
    // One trajectory for each vehicle in current state
    vector<vector<nbr_vehicle>> trajectories(current_state.size());

    // Iterate over all vehicles 
    for(int i=0;i<current_state.size();i++)
    {
        trajectories[i] = predict_single_vehicle_states(current_state[i],settings);
    }

    return trajectories;

}

//Variant of predict_nbr_trajectories, that retruns only the state at the end of the time window
vector<nbr_vehicle> Prediction::predict_nbr_final_states(const vector<nbr_vehicle>& current_state, config& settings)
{
    vector<vector<nbr_vehicle>> trajectories = predict_nbr_trajectories(current_state,settings);
    vector<nbr_vehicle> end_states(current_state.size());

    for(int i=0;i<trajectories.size();i++)
    {
        end_states[i] = trajectories[i][trajectories[i].size()-1];
    }

    return end_states;    
}

