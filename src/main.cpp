#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "PIDTrainer.hpp"
#include <math.h>
#include <cfloat>


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

  PID pid;
  PID speedPid;
  //Kp = 0.3, Ki = 0.0005, and Kd = 20.
  //pid.Init(0.3,0.0005, 20);
  pid.Init(0,0,0);
  PIDTrainer trainer(&pid, 0.05, 10);
  speedPid.Init(0,0,0);
  PIDTrainer speedTrainer(&speedPid, 0.1, 30);

  h.onMessage([&pid, &trainer,&speedPid, &speedTrainer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    static int i = 1;
    static int prev = 1;
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

          double tr = std::stod(j[1]["throttle"].get<std::string>());
          double steer_value;
          double throttle_val;
          if(!angle && !tr)
            std::cout << " Primo campione cte = " << cte << std::endl;
          else
            std::cout << " NON PRIMO CAMPIONE = " << angle << " " << tr << " " << cte << std::endl;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          // std::cout << " PID current coefficient Kp " << pid.getKp() << " Ki " <<
          //   pid.getKi() << " kd " << pid.getKd() << " current cte " << cte << std::endl;
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          //speedPid.UpdateError(cte);
          //throttle_val = speedPid.TotalError();
          //std::cout << "Steering value = " << steer_value << std::endl;
          if (steer_value > 1) {
            //std::cout << "CORRETTO ANGOLO a 1 " << std::endl;
            steer_value = 1;
          }
          if (steer_value < -1) {
            //std::cout << "CORRETTO ANGOLO a -1 " << std::endl;
            steer_value = -1;
          }

          //if(!pid.isTuned() || !speedPid.isTuned()) {
          if(!pid.isTuned()) {
            //std::cout << "My robot is running in tuning " << i << " samples processed " << std::endl;
            // Even if we reset the simulator some message still arrive..
            // I would like to filter them.

            //if(abs(cte) <= trainer.GetBestError()) {
              trainer.UpdateError(cte);
              //speedTrainer.UpdateError(cte);
              i++;
            //}
            // if (i > 1500) {
            //     std::cout << "We finished a loop (probably)" << std::endl;
            //     std::vector<double> best_coeff = trainer.dumpCoefficient();
            //     std::cout << "Here's the parameter kp " << best_coeff[0] << " Ki " << best_coeff[1] << " kd " << best_coeff[2] << std::endl;
            //     pid.setTuned(true);
            //     i = 0;
            // }
            // We are out now...:( need to reset simulator and restart tunin

            //if(abs(cte) > 0.8) {
            //    prev = i;
                trainer.TuneParameters();
                //speedTrainer.TuneParameters();
                //std::string msg("42[\"reset\", {}]");
                //ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                i = 1;
            //}
          } else {
            if(abs(cte) >=  0.8) {
              std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXX Start tuning again" << std::endl;
              pid.setTuned(false);
              std::string msg("42[\"reset\", {}]");
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
          }

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          if (speed <= 20) {
            msgJson["throttle"] = 0.3;
          } else {
            msgJson["throttle"] = -0.1;
          }

          // We got suck somewhere...:(
          if (speed <= 0.1 || abs(cte) > 1.5) {
            std::cout << " XXXXX RESET " << std::endl;
            std::string msg("42[\"reset\", {}]");
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
