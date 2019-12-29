#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"


#define  LANE_WIDTH           4 // meters
#define  MIDDLE_LANE_CENTER   6 // meters
#define  MPH_TO_MPS           1.0/2.237 // Mulitply with
#define  MPS_TO_MPH           2.237 // Mulitply with
#define  SPEED_LIMIT          49.5 // MPH
#define  SPEED_MAX            100.0 // MPH
#define  DISTANCE_MAX         50.0 // Meter
#define  NB_OF_LANES          3
#define  TOO_CLOSE_DISTANCE   30.0
#define  COST_MAX             1.0


#define OBJECT_ID             0
#define OBJECT_X              1
#define OBJECT_Y              2
#define OBJECT_VX             3
#define OBJECT_VY             4
#define OBJECT_S              5
#define OBJECT_D              6

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


struct lane_info
{
  double speed_veh_front;
  double speed_veh_back;
  double dist_veh_front;
  double dist_veh_back;
};


char* lane_to_name(int lane); // Used for debugging

void adaptVelInEgoLane(lane_info &egoLane, double &ref_vel);

double calcCostOfLaneChange(lane_info bestLane, double ref_vel);

void fillLanesInfo(const vector<vector<double>> &sensor_fusion, int prev_size, double car_s, lane_info lanesInfo[]);

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x, y;
    float s;
    float d_x, d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  int lane = 1; // Start in the middle lane
  double ref_vel = 0.0; // Vehicle starting velocity(MpH)
  string state = "KL"; // Keep lane

  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &state]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];

          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          // We do not want to crush into and pass by when possible.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();
          if (prev_size > 0)
          {
            car_s = end_path_s; // Predicted location of the ego car
          }

          lane_info lanesInfo[NB_OF_LANES];
          fillLanesInfo(sensor_fusion, prev_size, car_s, lanesInfo);

          lane_info egoLane = lanesInfo[lane];
          adaptVelInEgoLane(egoLane, ref_vel);

          // Calculate cost of lane change to the best lane
          double costs[NB_OF_LANES] = {COST_MAX, COST_MAX, COST_MAX};
          for (int l = 0; l < NB_OF_LANES ; l++)
          {
            lane_info &bestLane = lanesInfo[l];
            costs[l] = calcCostOfLaneChange(bestLane, ref_vel);
          }

          // Simple state machine of two states,
          // KL(keep lane) and CL(change lane)
          if (state == "KL")
          {
            double cost_min = costs[lane];
            for (int l = 0; l < NB_OF_LANES ; l++)
            {
              if (abs(lane - l) <= 1) // Do not change two lanes at a time
              {
                if (costs[l] < cost_min)
                {
                  cost_min = costs[l];
                  if (ref_vel > 20.0) // Do not change lane unless car has some speed
                  {
                    lane = l;
                    state = "LC";
                  }
                }
              }
            }
          }
          else if (state == "LC")
          {
            // Finish the maneuver then set state back to KL
            if (prev_size > 0)
            {
              if ((end_path_d>lane*LANE_WIDTH) && (end_path_d<(lane+1)*LANE_WIDTH))
              {
                if ((end_path_d>lane*LANE_WIDTH) && (end_path_d<(lane+1)*LANE_WIDTH))
                {
                  // Only when start and end point of the path are in the same lane
                  state = "KL";
                }
              }
            }
          }

          printf("---------------------------------------------------------------\n");
          printf ("%12s%10f, %10f, %10f\n", "Dis front:", lanesInfo[0].dist_veh_front, lanesInfo[1].dist_veh_front, lanesInfo[2].dist_veh_front);
          printf ("%12s%10f, %10f, %10f\n", "Dis back:", lanesInfo[0].dist_veh_back, lanesInfo[1].dist_veh_back, lanesInfo[2].dist_veh_back);
          // printf("...............................................................\n");
          printf ("%12s%10f, %10f, %10f\n", "Vel front:", lanesInfo[0].speed_veh_front, lanesInfo[1].speed_veh_front, lanesInfo[2].speed_veh_front);
          printf ("%12s%10f, %10f, %10f\n", "Vel back:", lanesInfo[0].speed_veh_back, lanesInfo[1].speed_veh_back, lanesInfo[2].speed_veh_back);
          printf ("%12s%10f, %10f, %10f\n", "Cost:", costs[0], costs[1], costs[2]);
          printf ("...............................................................\n");

          // Create a list of widely spaced waypoints, evenly spaced at 30 meters
          // Later we will interpolate these waypoints with a spline and fill it in with more points
          // that control speed
          vector<double> ptsx;
          vector<double> ptsy;


          // Refernce x, y, yaw states
          // Either we would refernce the starting point as where the car is or at the previous paths end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // std::cout << "Size of previous path: " << prev_size << std::endl;

          // If previous size is almost empty, use the car as starting reference.
          if (prev_size < 2)
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);


            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);

          }
          // Use previous path's end point as starting reference
          else
          {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            // Use two points that make the path tangent to the previous path's
            // end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

          }

          // In Frenet add evenly 30m seperated points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+30.0, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60.0, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90.0, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i = 0; i < ptsx.size(); i++)
          {

            // std::cout << ptsx[i] << "," << ptsy[i] << std::endl;
            // shift car reference angle to 0 degrees
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
            ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
          }

          // create a spline
          tk::spline spline;

          // set (x, y) to the spline
          spline.set_points(ptsx, ptsy);



          // Points used by the planner and will be sent to the simulator
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all the previous path points from the previous path
          for (int i = 0; i < previous_path_x.size(); ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at our
          // desired reference velocity
          double target_x = 30.0;
          double target_y = spline(target_x);
          double target_dist = sqrt((target_x*target_x)+(target_y*target_y));

          double x_add_on = 0.0;

          for (int i = 1; i <= 50-previous_path_x.size(); i++)
          {
            double N = target_dist / (0.02*ref_vel*MPH_TO_MPS);
            double x_point = x_add_on + target_x/N;
            double y_point = spline(x_point);

            x_add_on = x_point;

            // translate back to global coordinate
            double x_ref = x_point;
            double y_ref = y_point;

            x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
            y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

          }

          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}

void fillLanesInfo(const vector<vector<double>> &sensor_fusion, int prev_size, double car_s, lane_info lanesInfo[])
{
  for (int l = 0; l < NB_OF_LANES; l++)
  {
    lanesInfo[l].speed_veh_front = SPEED_MAX;
    lanesInfo[l].speed_veh_back = 0.0;
    lanesInfo[l].dist_veh_front = +DISTANCE_MAX;
    lanesInfo[l].dist_veh_back = -DISTANCE_MAX;
  }
  // Calculate speed of each lane
  for (int i = 0; i < sensor_fusion.size(); i++)
  {
    // Car is in my lane?
    double d = sensor_fusion[i][OBJECT_D];
    double vx = sensor_fusion[i][OBJECT_VX];
    double vy = sensor_fusion[i][OBJECT_VY];
    double check_speed = sqrt(vx*vx + vy*vy);
    double check_car_s = sensor_fusion[i][OBJECT_S];
    // Predict s in the future
    check_car_s += (double)prev_size*.02*check_speed;

    for (int l = 0; l < NB_OF_LANES; l++)
    {
      if ( (d>l*LANE_WIDTH) && (d<(l+1)*LANE_WIDTH))
      {
        // Get the nearest car to the ego vehicle in lane
        double delta_s = check_car_s - car_s;
        if (delta_s > 0) // In front of the ego vehicle
        {
          if (delta_s < lanesInfo[l].dist_veh_front)
          {
            lanesInfo[l].dist_veh_front = delta_s;
            lanesInfo[l].speed_veh_front = check_speed * MPS_TO_MPH; // Speed of nearest vehicle in the front
          }
        }
        else // Behind the ego vehicle
        {
          if (delta_s > lanesInfo[l].dist_veh_back)
          {
            lanesInfo[l].dist_veh_back = delta_s;
            lanesInfo[l].speed_veh_back = check_speed * MPS_TO_MPH; // Speed of nearest vehicle in the back
          }
        }
      }
    }
  }
}

char* lane_to_name(int lane)
{
  switch (lane)
  {
    case 0:
      return "Left";
    case 1:
      return "Middle";
    case 2:
      return "Right";
  }
}

void adaptVelInEgoLane(lane_info &egoLane, double &ref_vel)
{
  if (egoLane.dist_veh_front < TOO_CLOSE_DISTANCE)
  {
    printf("egoLane front(%f), back(%f) ref_vel(%f)\n", egoLane.speed_veh_front, egoLane.speed_veh_back, ref_vel);
    ref_vel -= 0.224;
  }
  else if (ref_vel < SPEED_LIMIT)
  {
    ref_vel += 0.224;
  }
}

double calcCostOfLaneChange(lane_info bestLane, double ref_vel)
{
  double cost;
  double s_front = bestLane.dist_veh_front;
  double s_back = bestLane.dist_veh_back;
  double v_front = bestLane.speed_veh_front;
  double v_back = bestLane.speed_veh_back;

  double cost_front, cost_back;

  // Front vehicle
  if ((s_front > TOO_CLOSE_DISTANCE)&&(v_front > ref_vel))
  {
    cost_front = 1/s_front; // Lower cost for lane with further car in the front
  }
  else
  {
    cost_front = COST_MAX / 2.0;
  }

  // Back vehicle
  if ((s_back < -TOO_CLOSE_DISTANCE)&&(ref_vel > v_back))
  {
    cost_back = abs(1/s_back); // Lower cost for lane with further car in the back
  }
  else
  {
    cost_back = COST_MAX / 2.0;
  }

  cost = cost_front + cost_back;
  cost = (cost > 0.5) ? COST_MAX : cost; // Normalize to COST_MAX(1.0)

  return cost;
}
