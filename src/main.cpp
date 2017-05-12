//======================================================================================================================
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "Color.hpp"
#include "ViewPort.hpp"
//======================================================================================================================
// for convenience
using json = nlohmann::json;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

//----------------------------------------------------------------------------------------------------------------------
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


//----------------------------------------------------------------------------------------------------------------------
// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++)
  {
    for (int i = 0; i < order; i++)
    {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


//----------------------------------------------------------------------------------------------------------------------

double polyeval(const Eigen::VectorXd& coeffs, double x)
{
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++)
  {
    result += coeffs[i] * pow(x, i);
  }

  return result;
}


//----------------------------------------------------------------------------------------------------------------------

static Eigen::VectorXd SFitPolynomial(const Eigen::MatrixXd aTransformedWayPoints)
{
  return polyfit(aTransformedWayPoints.col(0), aTransformedWayPoints.col(1), 3);
}


//----------------------------------------------------------------------------------------------------------------------

static Eigen::MatrixXd STransformWayPointsToVehicleCoordinates(
  const vector<double>& ptsx,
  const vector<double>& ptsy,
  double world_x,
  double world_y,
  double world_psi)
{
  const int n = ptsx.size();
  Eigen::MatrixXd Coords(n,2);
  Coords << Eigen::Map<const Eigen::VectorXd>(ptsx.data(), ptsx.size()),
            Eigen::Map<const Eigen::VectorXd>(ptsy.data(), ptsy.size());

  Coords.rowwise() -= Eigen::RowVector2d(world_x, world_y);

  Eigen::Matrix2d T;
  T << cos(-world_psi), -sin(-world_psi), -sin(-world_psi), -cos(-world_psi);

  return Coords * T.transpose();
}


//----------------------------------------------------------------------------------------------------------------------

static Eigen::VectorXd STransformState(double px, double py, double psi, double v)
{
  Eigen::VectorXd result(4);
  result << px, py, psi, v;
  return result;
}


//----------------------------------------------------------------------------------------------------------------------

Eigen::MatrixXd STransformToScreenCoordinates(const ViewPort& vp, const Eigen::MatrixXd& aCoords)
{
  const int n = aCoords.rows();
  Eigen::MatrixXd TransformedCoordinates(n, 2);
  const double kScale = 4.0;
  TransformedCoordinates.col(0) = vp.Width() / 2.0 * Eigen::VectorXd::Ones(n) + kScale * aCoords.col(1);
  TransformedCoordinates.col(1) = vp.Height() * Eigen::VectorXd::Ones(n) - kScale * aCoords.col(0);
  return TransformedCoordinates;
}


//----------------------------------------------------------------------------------------------------------------------

static void SDrawWayPoints(ViewPort& vp, const Eigen::MatrixXd& aCoords)
{
  vp.DrawMarkers(STransformToScreenCoordinates(vp, aCoords), TColor::SColorGreen(), 4);
}


//----------------------------------------------------------------------------------------------------------------------

static void SDrawFit(ViewPort& vp, const vector<double>& result)
{
  const int w = vp.Width();
  const int h = vp.Height();
  const double kScale = 4.0;
  vp.MoveTo(w/2, h-1);


  for (int i = 2; i < result.size(); i+=2)
  {
    const double x = w / 2 + kScale * result[i+1];
    const double y = h - kScale * result[i];
    const int kThickness = 2;
    vp.LineTo(x, y, TColor::SColorYellow(), kThickness);
  }
}


//----------------------------------------------------------------------------------------------------------------------

static void SDrawSteering(ViewPort& vp, double delta, double v)
{
  double psi = 0;
  double x = 0;
  double y = 0;
  const double Lf = 2.67;
  const double dt = 0.05;
  const int w = vp.Width();
  const int h = vp.Height();
  const double kScale = 4.0;
  vp.MoveTo(w/2, h-1);

  if (v < 0.25)
  {
    return;
  }

  double t = 0.0;

  while (t < 1.0)
  {
      x += v * cos(psi) * dt;
      y += v * sin(psi) * dt;
      psi += v/Lf * delta * dt;
      vp.LineTo(w/2 + kScale*y, h - kScale*x, TColor::SColorWhite(), 2);
      t += dt;
  }
}


//----------------------------------------------------------------------------------------------------------------------

static double SCalcCTE(const Eigen::VectorXd& aCoeffs)
{
  return polyeval(aCoeffs, 0);
}


//----------------------------------------------------------------------------------------------------------------------

static double SCalcOrientationError(const Eigen::VectorXd& aCoeffs)
{
  const double f0 = aCoeffs(1);
  return atan(f0);
}


//----------------------------------------------------------------------------------------------------------------------

int main()
{
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

#define ENABLE_VISUALIZATION
#if defined(ENABLE_VISUALIZATION)
  ViewPort vp(512,512,"MPC");
#else
  int vp;
#endif

  h.onMessage([&mpc, &vp](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          const vector<double> ptsx = j[1]["ptsx"];
          const vector<double> ptsy = j[1]["ptsy"];
          const double py = j[1]["y"];
          const double px = j[1]["x"];
          const double psi = j[1]["psi"];
          const double v = j[1]["speed"];

          const auto TransformedWayPoints = STransformWayPointsToVehicleCoordinates(ptsx, ptsy, px, py, psi);
          const auto coeffs = SFitPolynomial(TransformedWayPoints);
          const double cte = SCalcCTE(coeffs);
          const double epsi = SCalcOrientationError(coeffs);

          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          const auto result = mpc.Solve(state, coeffs);
          const double steer_angle = result[0];
          const double steer_value = 180 * steer_angle / M_PI / 25;
          const double throttle_value = result[1];


#if defined(ENABLE_VISUALIZATION)
          vp.Fill(TColor::SColorGray(32));
          SDrawWayPoints(vp, TransformedWayPoints);
          std::cout << TransformedWayPoints << std::endl;
          SDrawFit(vp, result);
          SDrawSteering(vp, steer_angle, v);
          const double xt = 10.0;
          const double yt = 10.0;
          Eigen::MatrixXd c(1,2);
          c << xt,yt;
          vp.DrawMarkers(STransformToScreenCoordinates(vp, c), TColor::SColorRed(), 4);
          vp.Show();
#endif

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
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

//======================================================================================================================
