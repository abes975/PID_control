#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "PIDTrainer.hpp"
#include <math.h>
#include <cfloat>
#include <limits>


// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;
  // Annoying stuff...even if the simulator is resetted socket is not flushed
  // So other samples could arrive...and we have to wait to re start tuning
  bool resetCompleted = true;
  double prev_best_error = FLT_MAX;
  PID pid;
  //Kp = 0.5 Kd = 7.75 Ki = 0.000244141 with constant throttle 0.3
  //pid.Init(0.5,0.000244141, 7.75);

  pid.Init(0,0,0);
  PIDTrainer trainer(&pid, 0.2);

  h.onMessage([&pid, &trainer,&resetCompleted, &prev_best_error](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // how many sample we use to tune parameter (how_many * scaling_factor)
    int max_drift = 2.7;
    int how_many = 500;
    static int scaling_factor = 1;
    static int samples = 1;
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          double trottle = std::stod(j[1]["throttle"].get<std::string>());

          double steer_value;
          double throttle_val;
          // Even after a reset has been issued some spurious sample (already
          // in the socket buffer might arrive..and we do not like them)
          if(!angle && !trottle && !resetCompleted) {
            resetCompleted = true;
            std::cout << " First good value = " << cte << std::endl;
          }
          if (resetCompleted)
            samples++;

          if(!pid.isTuned() && resetCompleted) {
            trainer.UpdateError(cte);

            if(samples == how_many * scaling_factor) {
              std::cout << " We got " << samples << " samples " << std::endl;
              trainer.TuneParameters();
              // if (trainer.GetBestError() <= prev_best_error) {
              //   std::cout << "\t\t\tWe want more samples " << std::endl;
              //   scaling_factor = 1.5;
              //   prev_best_error = trainer.GetBestError();
              // }
              std::string msg("42[\"reset\", {}]");
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              resetCompleted = false;
              samples = 1;
            }
          }
          if(pid.isTuned()) {
            std::cout << "we are running with Kp = " << pid.getKp() << " Kd = " <<
              pid.getKd() << " Ki = " << pid.getKi() << " cte " << cte << std::endl;
          }

          if (resetCompleted) {
            pid.UpdateError(cte);
            steer_value = pid.TotalError();

            if (steer_value > 1) {
              steer_value = 1;
            }
            if (steer_value < -1) {
              steer_value = -1;
            }

            // if(pid.isTuned() && ((fabs(cte) > max_drift) || speed <= 1))) {
            //     std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXX Start tuning again" << std::endl;
            //     pid.setTuned(false);
            //     std::string msg("42[\"reset\", {}]");
            //     ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            // }

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            if (speed <= 50) {
              if(!pid.isTuned())
                msgJson["throttle"] = 0.75;
              else
                msgJson["throttle"] = 0.3;
            } else {
              msgJson["throttle"] = -0.1;
            }

            // We got suck somewhere...:(
            if ((samples >= (how_many * scaling_factor)) && ((speed <= 0.1) || (fabs(cte) > max_drift))) {
              std::cout << " XXXXX RESET BECAUSE STUCK speed = " << speed << "max_error = " << fabs(cte) << std::endl;
              if(pid.isTuned()) {
                std::cout << "Stuck even if TUNED...no goood " << std::endl;
                pid.setTuned(false);
              }
              std::string msg("42[\"reset\", {}]");
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              resetCompleted = false;
              samples = 1;
            }

            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
