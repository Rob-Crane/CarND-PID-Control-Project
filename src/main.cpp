#include <math.h>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include <uWS/uWS.h>
#include "json.hpp"

#include "PID.h"
#include "PID_updater.h"

// for convenience
using nlohmann::json;
using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::steady_clock;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Set maximum cross-track-error before terminating training.
  const double MAX_CTE = 3.0;
  // Set initial parameter update vector.
  const ParamVector INITIAL_PARAM = {0.496113, 9.69385, 0.000253635};
  // Set initial parameter update vector.
  const ParamVector INITIAL_D_PARAM = {0.05, 0.3, 0.00005};
  // Set twiddle scaling factors.
  const float UPSCALE = 1.1;
  const float DOWNSCALE = 0.9;
  // Approx distance car should travel between param updates.
  const double UPDATE_DISTANCE = 2700.0;
  // Index of gain to evaluate first.
  const unsigned int INIT_I = 0;

  // Create with initial values.
  PID pid(INITIAL_PARAM);
  PIDUpdater updater(INITIAL_PARAM, INITIAL_D_PARAM, UPSCALE, DOWNSCALE,
                     INIT_I);
  double best_error = std::numeric_limits<double>::max();
  double error = 0.0;
  double distance = 0.0;

  auto last = steady_clock::now();
  bool clock_init = false;
  h.onMessage(
      [&pid, &updater, MAX_CTE, UPDATE_DISTANCE, &best_error, &error, &distance,
       &last, &clock_init](uWS::WebSocket<uWS::SERVER> ws, char *data,
                           size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message
        // event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
          auto s = hasData(string(data).substr(0, length));

          if (s != "") {
            auto j = json::parse(s);

            string event = j[0].get<string>();

            if (event == "telemetry") {
              // j[1] is the data JSON object
              double cte = std::stod(j[1]["cte"].get<string>());
              double speed = std::stod(j[1]["speed"].get<string>());

              if (clock_init) {
                auto now = steady_clock::now();
                microseconds dt = duration_cast<microseconds>(now - last);
                unsigned long ms = dt.count();
                distance += speed * double(ms) / 1E6;
              }
              error += cte * cte;
              bool update_ready = distance > UPDATE_DISTANCE;
              bool max_cte_exceed = fabs(cte) > MAX_CTE;
              if (update_ready || max_cte_exceed) {
                std::cout << "======" << std::endl;
                std::cout << "Eval complete: " << updater.desc() << " "
                          << updater.state_desc();
                if (update_ready) {
                  std::cout << " Diff From best: " << best_error - error
                            << std::endl;
                  if (error < best_error) {
                    best_error = error;
                    std::cout << "Was best!";
                    updater.good_outcome();
                  } else {
                    std::cout << std::endl;
                    updater.bad_outcome();
                  }
                } else {
                  updater.bad_outcome();
                  std::cout
                      << "Max CTE exceeded.  Restart simulator and press Enter."
                      << std::endl;
                  std::cin.get();
                }
                pid.UpdateGains(updater.p());
                std::cout << "Now Try: " << updater.desc() << " "
                          << updater.state_desc() << std::endl;
                distance = 0.0;
                error = 0.0;
              }
              double steer_value = -pid.UpdateError(cte);

              json msgJson;
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = 0.2;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              clock_init = true;
              last = steady_clock::now();
            }  // end "telemetry" if
          } else {
            // Manual driving
            string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }  // end websocket message if
      });  // end h.onMessage

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
