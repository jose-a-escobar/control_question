/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 				Aaron Brown
 **********************************************/

/**
 * @file main.cpp
 **/

#include <string>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <iostream>
#include <fstream>
#include <typeinfo>

#include "json.hpp"
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include "Eigen/QR"
#include "behavior_planner_FSM.h"
#include "motion_planner.h"
#include "planning_params.h"
#include "utils.h"
#include "pid_controller.h"

#include <limits>
#include <iostream>
#include <fstream>
#include <uWS/uWS.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <time.h>

using namespace std;
using json = nlohmann::json;

#define _USE_MATH_DEFINES

string hasData(string s) {
  auto found_null = s.find("null");
    auto b1 = s.find_first_of("{");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
      return "";
    }
    else if (b1 != string::npos && b2 != string::npos) {
      return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double angle_between_points(double x1, double y1, double x2, double y2){
  return atan2(y2-y1, x2-x1);
}

BehaviorPlannerFSM behavior_planner(
      P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
      P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
      P_MAX_ACCEL, P_STOP_LINE_BUFFER);

// Decalre and initialized the Motion Planner and all its class requirements
MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

bool have_obst = false;
vector<State> obstacles;

void path_planner(vector<double>& x_points, vector<double>& y_points, vector<double>& v_points, double yaw, double velocity, State goal, bool is_junction, string tl_state, vector< vector<double> >& spirals_x, vector< vector<double> >& spirals_y, vector< vector<double> >& spirals_v, vector<int>& best_spirals){

  State ego_state;

  ego_state.location.x = x_points[x_points.size()-1];
  ego_state.location.y = y_points[y_points.size()-1];
  ego_state.velocity.x = velocity;

  if( x_points.size() > 1 ){
  	ego_state.rotation.yaw = angle_between_points(x_points[x_points.size()-2], y_points[y_points.size()-2], x_points[x_points.size()-1], y_points[y_points.size()-1]);
  	ego_state.velocity.x = v_points[v_points.size()-1];
  	if(velocity < 0.01)
  		ego_state.rotation.yaw = yaw;

  }

  Maneuver behavior = behavior_planner.get_active_maneuver();

  goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state);

  if(behavior == STOPPED){

  	int max_points = 20;
  	double point_x = x_points[x_points.size()-1];
  	double point_y = y_points[x_points.size()-1];
  	while( x_points.size() < max_points ){
  	  x_points.push_back(point_x);
  	  y_points.push_back(point_y);
  	  v_points.push_back(0);

  	}
  	return;
  }

  auto goal_set = motion_planner.generate_offset_goals(goal);

  auto spirals = motion_planner.generate_spirals(ego_state, goal_set);

  auto desired_speed = utils::magnitude(goal.velocity);

  State lead_car_state;  // = to the vehicle ahead...

  if(spirals.size() == 0){
  	cout << "Error: No spirals generated " << endl;
  	return;
  }

  for(int i = 0; i < spirals.size(); i++){

    auto trajectory = motion_planner._velocity_profile_generator.generate_trajectory( spirals[i], desired_speed, ego_state,
                                                                                    lead_car_state, behavior);

    vector<double> spiral_x;
    vector<double> spiral_y;
    vector<double> spiral_v;
    for(int j = 0; j < trajectory.size(); j++){
      double point_x = trajectory[j].path_point.x;
      double point_y = trajectory[j].path_point.y;
      double velocity = trajectory[j].v;
      spiral_x.push_back(point_x);
      spiral_y.push_back(point_y);
      spiral_v.push_back(velocity);
    }

    spirals_x.push_back(spiral_x);
    spirals_y.push_back(spiral_y);
    spirals_v.push_back(spiral_v);

  }

  best_spirals = motion_planner.get_best_spiral_idx(spirals, obstacles, goal);
  int best_spiral_idx = -1;

  if(best_spirals.size() > 0)
  	best_spiral_idx = best_spirals[best_spirals.size()-1];

  int index = 0;
  int max_points = 20;
  int add_points = spirals_x[best_spiral_idx].size();
  while( x_points.size() < max_points && index < add_points ){
    double point_x = spirals_x[best_spiral_idx][index];
    double point_y = spirals_y[best_spiral_idx][index];
    double velocity = spirals_v[best_spiral_idx][index];
    index++;
    x_points.push_back(point_x);
    y_points.push_back(point_y);
    v_points.push_back(velocity);
  }


}

void set_obst(vector<double> x_points, vector<double> y_points, vector<State>& obstacles, bool& obst_flag){

	for( int i = 0; i < x_points.size(); i++){
		State obstacle;
		obstacle.location.x = x_points[i];
		obstacle.location.y = y_points[i];
		obstacles.push_back(obstacle);
	}
	obst_flag = true;
}

int main ()
{
  cout << "starting server" << endl;
  uWS::Hub h;

  double new_delta_time;
  int i = 0;

  fstream file_steer;
  file_steer.open("steer_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_steer.close();
  fstream file_throttle;
  file_throttle.open("throttle_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_throttle.close();

  time_t prev_timer;
  time_t timer;
  time(&prev_timer);

  // initialize pid steer
  /**
  * TODO (Step 1): create pid (pid_steer) for steer command and initialize values
  **/
  PID pid_steer = PID();

  // initialize pid throttle
  /**
  * TODO (Step 1): create pid (pid_throttle) for throttle command and initialize values
  **/

  PID pid_throttle = PID();
  //Initialize throttle PID ***********************************
  //Set throttle parameters
  
  double tau_p_throttle = 0.35;  
  double tau_d_throttle = 0.2;  
  double tau_i_throttle = 0.01;  
  double output_lim_max_throttle = 1.0;  
  double output_lim_min_throttle = -1.0;  
  //PID throttle:: Call Initialize function
  pid_throttle.Init(tau_p_throttle, tau_i_throttle, tau_d_throttle, output_lim_max_throttle, output_lim_min_throttle);  
  //End throttle PID Initialization ***********************************
  
  //Initialize steer PID ***********************************
  //Set steer parameters
  double tau_p_steer = 0.30;  
  double tau_d_steer = 0.4;  
  double tau_i_steer = 0.01;  
  double output_lim_max_steer = 1.2;  
  double output_lim_min_steer = -1.2;  
  //PID throttle:: Call Initialize function
  pid_steer.Init(tau_p_steer, tau_i_steer, tau_d_steer, output_lim_max_steer, output_lim_min_steer);

  h.onMessage([&pid_steer, &pid_throttle, &new_delta_time, &timer, &prev_timer, &i, &prev_timer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
        auto s = hasData(data);

        if (s != "") {

          auto data = json::parse(s);

          // create file to save values
          fstream file_steer;
          file_steer.open("steer_pid_data.txt");
          fstream file_throttle;
          file_throttle.open("throttle_pid_data.txt");

          vector<double> x_points = data["traj_x"];
          vector<double> y_points = data["traj_y"];
          vector<double> v_points = data["traj_v"];
          double yaw = data["yaw"];
          double velocity = data["velocity"];
          double sim_time = data["time"];
          double waypoint_x = data["waypoint_x"];
          double waypoint_y = data["waypoint_y"];
          double waypoint_t = data["waypoint_t"];
          bool is_junction = data["waypoint_j"];
          string tl_state = data["tl_state"];

          double x_position = data["location_x"];
          double y_position = data["location_y"];
          double z_position = data["location_z"];

          if(!have_obst){
          	vector<double> x_obst = data["obst_x"];
          	vector<double> y_obst = data["obst_y"];
          	set_obst(x_obst, y_obst, obstacles, have_obst);
          }

          State goal;
          goal.location.x = waypoint_x;
          goal.location.y = waypoint_y;
          goal.rotation.yaw = waypoint_t;

          vector< vector<double> > spirals_x;
          vector< vector<double> > spirals_y;
          vector< vector<double> > spirals_v;
          vector<int> best_spirals;

          path_planner(x_points, y_points, v_points, yaw, velocity, goal, is_junction, tl_state, spirals_x, spirals_y, spirals_v, best_spirals);

          // Save time and compute delta time
          time(&timer);
          new_delta_time = difftime(timer, prev_timer);
          prev_timer = timer;

          ////////////////////////////////////////
          // Steering control
          ////////////////////////////////////////

          /**
          * TODO (step 3): uncomment these lines
          **/
          // Update the delta time with the previous command
          pid_steer.UpdateDeltaTime(new_delta_time);

          // Compute steer error
          double error_steer;


          double steer_output;

          /**
          * TODO (step 3): compute the steer error (error_steer) from the position and the desired trajectory
          **/ 		  	  
		  double x_avg = 0.0;
		  double y_avg = 0.0;
		  double v_avg = 0.0;
		  double n_stations = (double)x_points.size();
		  
		  for (int i_check = 0; i_check < x_points.size(); i_check++) {
			
				x_avg += x_points[i_check];
				y_avg += y_points[i_check];
				v_avg += v_points[i_check];	
		  }
		  x_avg = x_avg/n_stations;
		  y_avg = y_avg/n_stations;
		  
		  double x_last = x_points[x_points.size() - 1];
		  double y_last = y_points[y_points.size() - 1];
		  double yaw_main = angle_between_points(x_avg, y_avg, x_last, y_last);
		  std::cout << "Yaw main = "<< yaw_main << std::endl;
		  
		  //Yaw compensation
		  double mag_main = sqrt( pow( (y_last - y_avg), 2.0) + pow( (x_last - x_avg), 2.0));
		  std::cout << "mag main = "<< mag_main << std::endl;
		  double V0 [2] = {0.0, 0.0}; //Unit vector from avg waypoint to last way point
		  double V1 [2] = {0.0, 0.0}; // Orthogonal vector to V0
		  V0[0] = (x_last - x_avg)/mag_main;
		  V0[1] = (y_last - y_avg)/mag_main;
		  V1[0] = - V0[1];
		  V1[1] =   V0[0];
		  std::cout << "V0_x = "<< V0[0] <<"V0_y = "<< V0[1] <<  std::endl;
		  std::cout << "V1_x = "<< V1[0] <<"V1_y = "<< V1[1] <<  std::endl;
		  
		  double V_distance [2] = {0.0, 0.0}; //Distance from actual location of vehicle to avg waypoint
		  V_distance[0] = (x_avg - x_position);
		  V_distance[1] = (y_avg - y_position);
		  std::cout << "x_position = "<< x_position <<"y_position = "<< y_position <<  std::endl;
		  std::cout << "V_distance[0] = "<< V_distance[0] <<"V_distance = "<< V_distance[1] <<  std::endl;
		  
		  double dot_yaw = V_distance[0] * V1[0] + V_distance[1] * V1[1]; //Dot product 
		  std::cout << "dot_yaw = "<< dot_yaw << std::endl;
		  //Calculate compensation
		  double max_yaw_compensation = M_PI * 0.25; //0.25; //Set Maximum yaw compensation 
		  double yaw_compensation = +dot_yaw * 0.25; //Only apply a fraction of the projection
		  if(yaw_compensation > max_yaw_compensation) yaw_compensation = max_yaw_compensation; //Cap the yaw compensation
          if(yaw_compensation < -max_yaw_compensation) yaw_compensation = -max_yaw_compensation; //Cap the yaw compensation
		  std::cout << "yaw_compensation = "<< yaw_compensation << std::endl;
		  
		  //Steering Compensation		  
		  double yaw_desired = yaw_main + yaw_compensation;
		  std::cout << "yaw_desired = "<< yaw_desired << std::endl;
		  
		  
          error_steer = yaw_desired - yaw;

          /**
          * TODO (step 3): uncomment these lines
          **/
          // Compute control to apply
          pid_steer.UpdateError(error_steer);
          steer_output = pid_steer.TotalError();

          // Save data
          file_steer.seekg(std::ios::beg);
          for(int j=0; j < i - 1; ++j) {
              file_steer.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          }
          file_steer  << i ;
          file_steer  << " " << error_steer;
          file_steer  << " " << steer_output << endl;

          ////////////////////////////////////////
          // Throttle control
          ////////////////////////////////////////

          /**
          * TODO (step 2): uncomment these lines
          **/
//           // Update the delta time with the previous command
          pid_throttle.UpdateDeltaTime(new_delta_time);

          // Compute error of speed
          double error_throttle;
          /**
          * TODO (step 2): compute the throttle error (error_throttle) from the position and the desired speed
          **/
		  
		  for (int i_test = 0; i_test < v_points.size(); i_test++) {
			std::cout << "i_test = " << i_test << " x_points = " << x_points[i_test] << " y_points = " << y_points[i_test] << std::endl;
			std::cout << "Car Velocity = " << v_points[i_test] << std::endl;
		  }
		  std::cout << "Actual position & Velocity of the car"<< std::endl;
		  std::cout << "x_position = " << x_position << " y_position = " << y_position << std::endl;
		  std::cout << "Actual Velocity " << velocity << std::endl;
		  
		  //std::cout << "Waypoints"<< std::endl;
		  //std::cout << "waypoint_x = " << waypoint_x << " waypoint_y = " << waypoint_y << std::endl;		  
		  
		  v_avg = v_avg/n_stations;
		  
		  std::cout << "Average Desired Velocity"<< std::endl;
		  std::cout << "n_stations = " << n_stations << " velocity = " << v_avg << std::endl;
          double dist_to_path = sqrt( pow( (x_points[0] - x_position), 2.0) + pow( (y_points[0] - y_position), 2.0) );
          //std::cout << "First point of the trajectory"<< std::endl;
          //std::cout << "x_points[0] = " << x_points[0] << " y_points[0] = " << y_points[0] << std::endl;
          std::cout << "Last point of the trajectory"<< std::endl;
          std::cout << "x_points[v_points.size()-1] = " << x_points[v_points.size()-1] << " y_points[v_points.size()-1] = " << y_points[v_points.size()-1] << std::endl;
          std::cout << "Distance from car to first pathpoint " << dist_to_path << std::endl;
		  std::cout << "***********      ************" << std::endl;
		  //Velocity Compensation
		  double dot_vel = V_distance[0] * V0[0] + V_distance[1] * V0[1]; //Dot product 
		  double max_vel_compensation = 1.5; //Half the max speed
		  double vel_compensation = -dot_vel * 0.15; //0.25; //0.75; // * 0.5; //Only apply a fraction of the projection
		  if(vel_compensation > max_vel_compensation) vel_compensation = max_vel_compensation; //Cap the throttle compensation
          if(vel_compensation < -max_vel_compensation) vel_compensation = -max_vel_compensation; //Cap the throttle compensation
		  
		  double vel_desired = v_avg + vel_compensation ;
		  		  
          // modify the following line for step 2
          error_throttle = vel_desired - velocity;

          double throttle_output;
          double brake_output;

          /**
          * TODO (step 2): uncomment these lines
          **/
          // Compute control to apply
          pid_throttle.UpdateError(error_throttle);
          double throttle = pid_throttle.TotalError();

          // Adapt the negative throttle to break
          if (throttle > 0.0) {
            throttle_output = throttle;
            brake_output = 0;
          } else {
            throttle_output = 0;
            brake_output = -throttle;
          }

          // Save data
          file_throttle.seekg(std::ios::beg);
          for(int j=0; j < i - 1; ++j){
              file_throttle.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
          }
          file_throttle  << i ;
          file_throttle  << " " << error_throttle;
          file_throttle  << " " << brake_output;
          file_throttle  << " " << throttle_output << endl;


          // Send control
          json msgJson;
          msgJson["brake"] = brake_output;
          msgJson["throttle"] = throttle_output;
          msgJson["steer"] = steer_output;

          msgJson["trajectory_x"] = x_points;
          msgJson["trajectory_y"] = y_points;
          msgJson["trajectory_v"] = v_points;
          msgJson["spirals_x"] = spirals_x;
          msgJson["spirals_y"] = spirals_y;
          msgJson["spirals_v"] = spirals_v;
          msgJson["spiral_idx"] = best_spirals;
          msgJson["active_maneuver"] = behavior_planner.get_active_maneuver();

          //  min point threshold before doing the update
          // for high update rate use 19 for slow update rate use 4
          msgJson["update_point_thresh"] = 16;

          auto msg = msgJson.dump();

          i = i + 1;
          file_steer.close();
          file_throttle.close();

      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

    }

  });


  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
      cout << "Connected!!!" << endl;
    });


  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
    {
      ws.close();
      cout << "Disconnected" << endl;
    });

  int port = 4567;
  if (h.listen("0.0.0.0", port))
    {
      cout << "Listening to port " << port << endl;
      h.run();
    }
  else
    {
      cerr << "Failed to listen to port" << endl;
      return -1;
    }


}
