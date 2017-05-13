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
  // Annoying stuff...even if the simulator is resetted socket is not flushed
  // So other samples could arrive...and we have to wait to re start tuning
  bool resetCompleted = true;

  PID pid;
  PID speedPid;

  //pid.Init(0.25,0.0,1.3);

  pid.Init(0,0,0);
  PIDTrainer trainer(&pid, 0.4);
  //pid.setTuned(true);

  speedPid.Init(0,0,0);
  PIDTrainer speedTrainer(&speedPid, 0.4);

  h.onMessage([&pid, &trainer,&resetCompleted, &speedPid, &speedTrainer]
      (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // how many sample we use to tune parameter (how_many * scaling_factor)
    int target_speed = 40;
    int max_drift = 3;
    int min_samples = 200;
    int how_many = 200;
    static double scaling_factor = 1;
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
          double speed_cte = 0;
          // Even after a reset has been issued some spurious sample (already
          // in the socket buffer might arrive..and we do not like them)
          if(!angle && !trottle && !resetCompleted) {
            resetCompleted = true;
            std::cout << " Primo campione cte = " << cte << std::endl;
          }

          // Run Twiddle
          if(resetCompleted) {
            speed_cte = target_speed - speed;
            if(!pid.isTuned() || !speedPid.isTuned()) {
              trainer.UpdateError(cte);
              std::cout << "Ma merda = " << speed_cte << std::endl;
              speedTrainer.UpdateError(speed_cte);
              samples++;
            }
            int total = int(how_many * scaling_factor);
            if(samples == total) {
              // std::cout << " We got " << samples << " samples " << std::endl;
              // if(trainer.getBestError() <= 0.2) {
              //   scaling_factor *= 1.5;
              //   std::cout << " We want more samples here " << int(how_many * scaling_factor) << std::endl;
              // }
              trainer.TuneParameters();
              std::cout << "TUNING PHASE: STEERING PID current coefficient Kp " << pid.getKp()
                 << " Kd " << pid.getKd() << " Ki " << pid.getKi() << " current cte "
                 << cte << std::endl;
              speedTrainer.TuneParameters();
              std::cout << "TUNING PHASE: SPEED PID current coefficient Kp " << speedPid.getKp()
                 << " Kd " << speedPid.getKd() << " Ki " << speedPid.getKi() << " current cte "
                 << cte << std::endl;
              std::string msg("42[\"reset\", {}]");
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              resetCompleted = false;
              samples = 1;
            }
          }

          if(resetCompleted && pid.isTuned()) {
            std::cout << "we are running STEERING PID with Kp = " << pid.getKp() << " Kd = " <<
              pid.getKd() << " Ki = " << pid.getKi()<< std::endl;
          }
          if(resetCompleted && speedPid.isTuned()) {
            std::cout << "we are running SPEED PID with Kp = " << speedPid.getKp() << " Kd = " <<
              speedPid.getKd() << " Ki = " << speedPid.getKi() << std::endl;
          }

          if(resetCompleted) {
            pid.UpdateError(cte);
            steer_value = pid.TotalError();
            speedPid.UpdateError(speed_cte);
            throttle_val = -speedPid.TotalError();

            if (steer_value > 1) {
              steer_value = 1;
            }
            if (steer_value < -1) {
              steer_value = -1;
            }
            if (throttle_val > 1) {
              throttle_val = 1;
            }
            if (throttle_val < -1) {
              throttle_val = -1;
            }

          }
          // // Do we need to go back to tuning mode?
          // if(resetCompleted && pid.isTuned() && fabs(cte) > 2 * max_drift) {
          //     std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXX WE DID NOT DO A GOOD JOB: reset = " << resetCompleted <<  " fabs(cte) = " << fabs(cte) << " Start tuning again" << std::endl;
          //     pid.setTuned(false);
          //     resetCompleted = false;
          //     samples = 1;
          //     std::string msg("42[\"reset\", {}]");
          //     ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          // }


          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_val;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          // We got suck somewhere...:(
          //std::cout << "ResetCompleted " << resetCompleted << " samples = " << samples << " min samples " << min_samples
          //<< " speed = " << speed << " fabs(cte) " << fabs(cte) << std::endl;
          if ((resetCompleted && (samples > (min_samples))) && (speed <= 0.1 || fabs(cte) >= max_drift)) {
            std::cout << " XXXXX RESET BECAUSE STUCK after " << samples << "cte = " << fabs(cte) << std::endl;
            // The car crahed after few samples...so..set a big error in order
            // to make twiddle to do a step back :)
            // This is an artifacts to penalize the facte we get off the road
            // before we got all the samples...
            // for(int i = samples; i < (how_many * scaling_factor); i++)
            //   trainer.UpdateError(cte);
            // samples = (how_many * scaling_factor) - 1;
            //trainer.TuneParameters();
            resetCompleted = false;
            samples = 1;
            std::string msg("42[\"reset\", {}]");
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }

          // auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          // ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
