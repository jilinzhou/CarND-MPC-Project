#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() {
  return M_PI;
}

double deg2rad(double x) {
  return x * pi() / 180;
}

double rad2deg(double x) {
  return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1         = s.find_first_of("[");
  auto b2         = s.rfind("}]");

  if (found_null != string::npos) {
    return "";
  } else if ((b1 != string::npos) && (b2 != string::npos)) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;

  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q      = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  // std::chrono::steady_clock::time_point prev =
  // std::chrono::steady_clock::now();

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER>ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;

    if ((sdata.size() > 2) && (sdata[0] == '4') && (sdata[1] == '2')) {
      string s = hasData(sdata);

      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // reference trajectory

          vector<double>ptsx = j[1]["ptsx"];
          vector<double>ptsy = j[1]["ptsy"];

          // current car position, heading, speed, steering angle, and throttle
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          v *= 0.44704; // convert it to m/s
          delta *= -1.0; // make counterclockwise +

          size_t n_wpts = ptsx.size();

          // convert reference trajectory waypoints to local car frame
          std::vector<double>wpts_x_local(n_wpts);
          std::vector<double>wpts_y_local(n_wpts);
          double delta_x;
          double delta_y;

          for (int i = 0; i < ptsx.size(); i++) {
            delta_x = ptsx[i] - px;
            delta_y = ptsy[i] - py;
            wpts_x_local[i] = delta_x * cos(psi) + delta_y * sin(psi);
            wpts_y_local[i] = -delta_x * sin(psi) + delta_y * cos(psi);
          }

          // fit waypoints into a 3rd polynomial
          Eigen::Map<Eigen::VectorXd>x_vals(&wpts_x_local[0], n_wpts);
          Eigen::Map<Eigen::VectorXd>y_vals(&wpts_y_local[0], n_wpts);
          auto coeffs = polyfit(x_vals, y_vals, 3);

          // Current states in car's local coordinate
          double x0 = 0.0;
          double y0 = 0.0;
          double psi0 = 0.0;
          double v0 = v;
          double delta0 = delta;
          double cte0 = y0 - coeffs[0];
          double epsi0 = psi0 - atan(coeffs[1]);

          // Take into account of latency so we move the car to a future state
          // with the help of kinematic model and compute control input for
          // that.
          double latency = 0.1;

          //   double x1 = x0 + (v * cos(psi0) * latency);
          double x1 = x0 + v0 * latency;

          //   double y1 = y0 + (v * sin(psi0) * latency);
          double y1 = y0;
          double psi1 = psi0 + (v0 * delta0 * latency / mpc.Lf);
          double v1 = v0 + a * latency;

          double f1 = ((coeffs[3] * x1 + coeffs[2]) * x1 + coeffs[1]) * x1  +
                      coeffs[0];
          double cte1 = f1 - y1;
          double epsi1 = epsi0 + v0 * delta0 * latency / mpc.Lf;

          Eigen::VectorXd state(6);

            // state << 0, 0, 0, v0*0.44704, cte0, epsi0;
          state << x1, y1, psi1, v1, cte1, epsi1;

          // solve for next control inputs
          //   std::chrono::steady_clock::time_point begin =
          // std::chrono::steady_clock::now();
          auto vars = mpc.Solve(state, coeffs);

          //   std::chrono::steady_clock::time_point end =
          // std::chrono::steady_clock::now();
          //   std::cout << "Time difference = " <<
          //   std::chrono::duration_cast<std::chrono::microseconds>(
          //     end - begin).count() << std::endl;

          // first three elements returned from the solver
          double cost = vars[0];
          double steer_value = vars[1];
          double throttle_value = vars[2];
          int offset = 3;
          int steps = mpc.N;

          json msgJson;

          // NOTE: Remember to divide by deg2rad(25) before you send the
          // steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25]
          // instead of [-1, 1].
          msgJson["steering_angle"] = -steer_value / (deg2rad(25));
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory
          vector<double>mpc_x_vals(mpc.N);
          std::copy(vars.begin() + offset, vars.begin() + offset + steps,
                    mpc_x_vals.begin());
          vector<double>mpc_y_vals(mpc.N);
          std::copy(vars.begin() + offset + steps, vars.end(),
                    mpc_y_vals.begin());

          // .. add (x,y) points to list here, points are in reference to the
          // vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double>next_x_vals = wpts_x_local;
          vector<double>next_y_vals = wpts_y_local;

          // .. add (x,y) points to list here, points are in reference to the
          // vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          //   std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER>ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER>ws, int code,
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
