#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// States to store the state about the lane
enum State {
  KEEP_LANE = 0,
  CHANGE_LANE = 1
};

// States to store the state about the velocity
enum SpeedState {
  FOLLOW_CAR = 0,
  USE_REFERENCE_VELOCITY = 1
};

// A structure to store the attributes of a close car
struct CloseCar {
  bool safe_to_drive;
  double s;
  double velocity;
};

// The reference velocity is a little lower than the maximum veloticy of 50 mph
const double kReferenceVelocity = 49.5; //mph
static State current_car_state = KEEP_LANE;
static SpeedState speed_state = USE_REFERENCE_VELOCITY;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y) {

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y) {

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y) {
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

double GetOptimalVelocity(double current_velocity, double reference_velocity) {
  if(abs(current_velocity - reference_velocity) < 0.45)
    // Adapt to the reference velocity but drive a little slower to regain some distance again
    return reference_velocity - 0.1;
  if(current_velocity < reference_velocity)
    // Accelerate
    return current_velocity + 0.45;
  // Slow down
  return current_velocity - 0.45;
}

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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  // The car starts in lane 1
  int lane = 1;

  // The car starts with a reference velocity of 0 mph since it stands still on the street at the beginning
  // The velocity will increase slowly in every step
  double ref_velocity = 0.0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&ref_velocity,&max_s](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

          	// NEW CODE STARTS HERE

          	// Store the number of the previous values of the previous path
          	int prev_size = previous_path_x.size();

          	// Choose the path's end as the reference s. If no path exists yet, choose the car's s as a reference s value.
          	double ref_s = car_s;
          	if(prev_size > 0) {
          	  ref_s = end_path_s;
          	}

          	// Create one close car for each lane and store it in a vector.
          	CloseCar close_car_1;
          	close_car_1.safe_to_drive = true;
          	close_car_1.s = numeric_limits<double>::max();
          	close_car_1.velocity = -1.0;
          	CloseCar close_car_2;
          	close_car_2.safe_to_drive = true;
          	close_car_2.s = numeric_limits<double>::max();
          	close_car_2.velocity = -1.0;
          	CloseCar close_car_3;
          	close_car_3.safe_to_drive = true;
          	close_car_3.s = numeric_limits<double>::max();
          	close_car_3.velocity = -1.0;
          	vector<CloseCar> close_cars = {close_car_1, close_car_2, close_car_3};

          	// Find out which car is closest to our car in each lane and store it in the close_cars vector
          	for(int i = 0; i < sensor_fusion.size(); i++) {
          	  // Figure out in which lane the other car is
          	  int close_car_lane;
          	  double d = sensor_fusion[i][6];
          	  if(d < 4)
          	    close_car_lane = 0;
          	  else if(d < 8)
          	    close_car_lane = 1;
          	  else
          	    close_car_lane = 2;

          	  // Calculate the speed of the other car
          	  double vx = sensor_fusion[i][3];
          	  double vy = sensor_fusion[i][4];
          	  double close_car_velocity = sqrt(vx * vx + vy * vy) * 2.24; // 1 m/s is about 2.24mph

          	  // Get the s value of the other car
          	  double close_car_s = sensor_fusion[i][5];

          	  // Check if the car is close the end of the track (max_s)
          	  // Since s will soon start from 0 again, max_s needs to be added to the cars which are already behind max_s.
          	  if(car_s > max_s - 500 && close_car_s < 500) {
          	    close_car_s += max_s;
          	  } else if(close_car_s > max_s - 500 && car_s < 500) { // If the already car is behind max_s, lower the other car's value.
          	    close_car_s -= max_s;
          	  }

          	  // Check if the other car's s value is close to our car's s value and if it is the closest car in its lane
          	  if(close_car_s > car_s - 10) {
          	    if(close_cars[close_car_lane].s > close_car_s) {
          	      close_cars[close_car_lane].velocity = close_car_velocity;
          	      close_cars[close_car_lane].s = close_car_s;
          	      close_cars[close_car_lane].safe_to_drive = close_car_s > ref_s + 10;
          	    }
          	  }
          	}

          	// Check if our car finished changing lanes and change the state
          	if(current_car_state == CHANGE_LANE && abs(2 + 4 * lane - car_d) < 1.0) {
          	  current_car_state = KEEP_LANE;
          	}

          	// If a cars appears on our current lane, search for a better lane and try to switch
         	if(current_car_state == KEEP_LANE && abs(close_cars[lane].s - car_s) < 100.0) {

         	  // Find the optimal lane
         	  int optimal_lane;
         	  if(close_cars[1].s == numeric_limits<double>::max()) {
         	    // If there is no car in lane 1, it is the optimal lane because is has two options to change the lane
         	    optimal_lane = 1;
         	  } else {
         	    // Find the lane with the furthest away car
                double closest_car_s = -1.0;
                for(int i = 0; i < close_cars.size(); i++) {
                  if(closest_car_s < close_cars[i].s) {
                    closest_car_s = close_cars[i].s;
                    optimal_lane = i;
                  }
                }
              }

              // Check if the car is not in the optimal lane and fast enough to switch lanes
         	  // (We don't want to surprise other cars coming from behind us)
         	  if(optimal_lane != lane && car_speed > 30.0) {
         	    // Try to switch to the optimal lane
         	    if(lane == 1 && close_cars[optimal_lane].safe_to_drive) {
         	      lane = optimal_lane;
         	      current_car_state = CHANGE_LANE;
         	      speed_state = USE_REFERENCE_VELOCITY;
         	    } else if(close_cars[1].safe_to_drive) {
         	      // The car needs to change to lane 1, no matter if it is currently in the left or right lane
         	      lane = 1;
         	      current_car_state = CHANGE_LANE;
         	      speed_state = USE_REFERENCE_VELOCITY;
         	    }
         	  }
          	}

          	if(abs(close_cars[lane].s - car_s) > 30) {
          	  // No car in our lane is close to us, so drive with reference velocity
          	  ref_velocity = GetOptimalVelocity(ref_velocity, kReferenceVelocity);
          	} else {
          	  // Slow down and follow the car in front
          	  speed_state = FOLLOW_CAR;
          	  ref_velocity = GetOptimalVelocity(ref_velocity, close_cars[lane].velocity);
          	}

          	// Create a list of widely spaced (x,y) waypoints for the spline
          	vector<double> ptsx;
          	vector<double> ptsy;

          	// Reference x, y and yaw starting points
          	double ref_x = car_x;
          	double ref_y = car_y;
          	double ref_yaw = deg2rad(car_yaw);

          	// If previous size is almost empty, use the car as starting reference
          	if(prev_size < 2) {

          	  // Calculate the cars previous x and y position to create a path tangent to the car
          	  double prev_car_x = car_x - cos(car_yaw);
          	  double prev_car_y = car_y - sin(car_yaw);

          	  ptsx.push_back(prev_car_x);
              ptsy.push_back(prev_car_y);

          	  ptsx.push_back(car_x);
          	  ptsy.push_back(car_y);

          	} else {

          	  // Use the previous path's end point as starting reference
          	  // Redefine reference state as previous path end point
          	  ref_x = previous_path_x[prev_size-1];
          	  ref_y = previous_path_y[prev_size-1];

          	  double previous_ref_x = previous_path_x[prev_size-2];
          	  double previous_ref_y = previous_path_y[prev_size-2];
          	  ref_yaw = atan2(ref_y - previous_ref_y, ref_x - previous_ref_x);

          	  // Use two points that make the path tangent to the previous path's end point
          	  ptsx.push_back(previous_ref_x);
          	  ptsy.push_back(previous_ref_y);

              ptsx.push_back(ref_x);
          	  ptsy.push_back(ref_y);
          	}

          	// Add points for the spline every 30m to get five points in total
          	vector<double> next_wp0 = getXY(ref_s + 30, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_wp1 = getXY(ref_s + 60, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_wp2 = getXY(ref_s + 90, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          	ptsx.push_back(next_wp0[0]);
            ptsy.push_back(next_wp0[1]);

          	ptsx.push_back(next_wp1[0]);
          	ptsy.push_back(next_wp1[1]);

            ptsx.push_back(next_wp2[0]);
          	ptsy.push_back(next_wp2[1]);

          	// Shift and rotate the points, so that the reference angle equals 0 degrees and heads along the x axis
          	for (int i = 0; i < ptsx.size(); i++) {
          	  double shift_x = ptsx[i] - ref_x;
          	  double shift_y = ptsy[i] - ref_y;

          	  ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
          	  ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          	}

          	// Calculate a spline which leads through the five waypoints
          	tk::spline s;
          	s.set_points(ptsx, ptsy);

          	// Create a vector for the waypoints which are used for our car's trajectory
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	// Add all of the previous waypoints from the previous trajectory to the new trajectory
          	for(int i = 0; i < previous_path_x.size(); i++) {
          	  next_x_vals.push_back(previous_path_x[i]);
          	  next_y_vals.push_back(previous_path_y[i]);
          	}

          	// Calculate how to break up spline points so that the car travels at the desired velocity
          	double target_x = 30.0;
          	double target_y = s(target_x);
          	double target_distance = sqrt(target_x * target_x + target_y * target_y);
          	double step_x = target_x / target_distance * (.02 * ref_velocity / 2.24); //  1 m/s is about 2.24mph


          	// Add additional waypoints to our cars trajectory until the trajectory consists of 50 waypoints
          	double transformed_x_point = 0.0;
          	double transformed_y_point = 0.0;
            for (int i = 1; i <= 50 - previous_path_x.size(); i++) {

          	  transformed_x_point += step_x;
          	  transformed_y_point = s(transformed_x_point);

          	  // Rotate back to normal after rotation it earlier
          	  double x_point = transformed_x_point * cos(ref_yaw) - transformed_y_point * sin(ref_yaw);
          	  double y_point = transformed_x_point * sin(ref_yaw) + transformed_y_point * cos(ref_yaw);

          	  // Shift back using the original reference point
          	  x_point += ref_x;
          	  y_point += ref_y;

          	  next_x_vals.push_back(x_point);
          	  next_y_vals.push_back(y_point);
          	}

            // HERE ENDS MY CODE

            msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
