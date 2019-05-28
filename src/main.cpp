#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
  double ref_speed = 0;    // Initial velocity
  int my_lane      = 1;    // Start in the middle lane
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
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
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
  
 // double ref_speed = 0;    // Initial velocity
  //int my_lane      = 1;    // Start in the middle lane
  ref_speed     = 0;
  my_lane      = 1; 
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          auto sensor_fusion = j[1]["sensor_fusion"];

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          int prev_size = previous_path_x.size();

          if(prev_size > 0)
          {
            car_s = end_path_s;
          }

          bool car_ahead = false;
          bool car_right = false;
          bool car_left  = false;


          // 1. Analyse sensor fusion data to detect vehicles in the vicinity
          for(int i = 0; i < sensor_fusion.size(); i++)
          {
            float d = sensor_fusion[i][6];
            int check_lane;

            // find the lane of the detected vehicle, note that there are 3 lanes with width 4m
            check_lane = d/4;
            
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_s = sensor_fusion[i][5];
            
            // Estimate car s position after executing previous trajectory.
            check_s += ((double)prev_size * 0.02 * check_speed);

            if((check_s > car_s) && ((check_s - car_s + 5) < 30 ))
            {
              // The check car is within next 30m of my car
              
              // Compare ego vehicle lane and my vehicle lane
              if((my_lane - check_lane) == 0)
              {
                car_ahead = true;
              }
              else if((my_lane - check_lane) == 1)
              {
                car_right = true;
              }
              else if((my_lane - check_lane) == -1)
              {
                car_left = true;
              }
            }
          }

          // 2. Plan behavior
          bool too_close = false;
          if(car_ahead == true)
          {
            // Change lane to the left
            if((car_left == false) && (my_lane != 2))
            {
              my_lane += 1;
            }
            // change lane to the right
            else if((car_right == false) && (my_lane != 0))
            {
              my_lane -= 1;
            }
            // if no room to change lane then slow down the vehicle
            else
            {
              too_close = true;
            }
          }
        
          // 3. Speed adjust
          if(too_close)
          {
              ref_speed -= 0.224; 
          }
          else if(ref_speed < 49.5)
          {
              ref_speed += 0.224;
          }

          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // later we will interpolate these waypoints with a spline and fill it in with more
          // points that control speed
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x, y, yar 
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous size is almost empty, use the car as starting reference
          if(prev_size < 2)
          {
            // Use two points that make the path tangent to the car
            double prev_x = car_x - cos(car_yaw);
            double prev_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_y);
            ptsy.push_back(car_y);
          }
          else
          {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            ptsx.push_back(previous_path_x[prev_size-2]);
            ptsx.push_back(previous_path_x[prev_size-1]);

            ptsy.push_back(previous_path_y[prev_size-2]);
            ptsy.push_back(previous_path_y[prev_size-1]);
          }

          // Setting target points in the future.
          vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*my_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*my_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*my_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // shift car reference angle to 0 degrees
          for ( int i = 0; i < ptsx.size(); i++ ) 
          {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }
          
          // Create planner points 
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // add previous points
          for ( int i = 0; i < prev_size; i++ ) 
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // define spline
          tk::spline s;
          s.set_points(ptsx, ptsy);

          //
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;
          // fill rest of the points with spline
          for( int i = 1; i < 50 - prev_size; i++ ) 
          {
            double N = target_dist/(0.02*ref_speed/2.24);
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          /* Irappa: End of Logic */

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