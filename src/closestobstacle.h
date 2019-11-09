/** Calculates the CoM of the closest obstacle for each leg by comparing fore-aft distance of each robot hip and each obstacle.
Written by Anmol Kathail (anmolk@seas.upenn.edu) **/

#pragma once

float dist(double obs, double logpos){
    return std::pow(obs - logpos, 2);
}

void closestObstacle(double LF_x, double RF_x, double RR_x, double LR_x, int* LF_obs, int* RF_obs, int* RR_obs, int* LR_obs){

    float min_LF = INT_MAX;
    float min_RF = INT_MAX;
    float min_RR = INT_MAX;
    float min_LR = INT_MAX;

    for(int i = 0; i < 15; i++){

        if(dist(LF_x,log_positions[i])< min_LF){
            *LF_obs = i+1;
            min_LF = dist(LF_x,log_positions[i]);
        }

        if(dist(RF_x,log_positions[i]) < min_RF){
            *RF_obs = i+1;
            min_RF = dist(RF_x,log_positions[i]);
        }

        if(dist(RR_x,log_positions[i]) < min_RR){
            *RR_obs = i+1;
            min_RR = dist(RR_x,log_positions[i]);
        }

        if(dist(LR_x,log_positions[i]) < min_LR){
            *LR_obs = i+1;
            min_LR = dist(LR_x,log_positions[i]);
        }
    }
}
