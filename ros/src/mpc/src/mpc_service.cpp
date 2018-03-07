#include "ros/ros.h"
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "mpc/mpc.h"


const double Lf = 2.67;
extern size_t  N;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


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


  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


// MPC callback
//////////////////////////////////////////////////////////////////
bool mpc_cb(mpc::mpc::Request &req, mpc::mpc::Response &res){
  vector<double> ptsx = req.ptsx;
  vector<double> ptsy = req.ptsy;
  
  double px = req.px;
  double py = req.py;
  double psi = req.psi;
  double v = req.speed;

  double steer_value = req.steering;
  double throttle_value = req.throttle;



  //  MPC initialized
  MPC mpc_agent;
  
  
  vector<double> next_x_val;
  vector<double> next_y_val;
  
  // Global to vehicle coordinate transformation
  Eigen::VectorXd _ptsx(ptsx.size());
  Eigen::VectorXd _ptsy(ptsy.size());
  for (int i = 0; i < ptsx.size(); ++i){
    _ptsx[i] = (ptsx[i] - px)  * cos(psi) + (ptsy[i] - py) * sin(psi);
    _ptsy[i] = -(ptsx[i] - px) * sin(psi) + (ptsy[i] - py) * cos(psi);
    
    next_x_val.push_back(_ptsx[i]);
    next_y_val.push_back(_ptsy[i]);
  }


  // Account for delay
  double latency = 0.6; // 100 mili sec.

  double delta = - steer_value;
  double npx = px + v * cos(psi) * latency;
  double npy = py + v * sin(psi) * latency;
  double npsi = psi + v * (delta / Lf) * latency;
  double nv = v + throttle_value * latency;

  // convert states to vehicle space
  double xd = npx - px;
  double yd = npy-  py;
  double x = xd * cos(psi) + yd * sin(psi);
  double y = yd * cos(psi) - xd * sin(psi);
  v = nv;
  psi = npsi - psi;


  auto coeffs = polyfit(_ptsx, _ptsy, 3);
  
  double cte = polyeval(coeffs, x) - y;
  double epsi = psi - atan(coeffs[1]);
  
  
  /*
  double wp_angle_x = 0.1;
  double wp_angle_y = 0.0;
  
  if (100 < _ptsx.size())
  {
      wp_angle_x  = _ptsx[100] - _ptsx[0];
      wp_angle_y  = _ptsy[100] - _ptsy[0];
  }
  double vector_to_wp_x = _ptsx[0] - x;
  double vector_to_wp_y = _ptsy[0] - y;
  
  double len_wp_angle = sqrt(wp_angle_x * wp_angle_x + wp_angle_y * wp_angle_y);
  double len_vector_to_wp = sqrt(vector_to_wp_x * vector_to_wp_x + vector_to_wp_y * vector_to_wp_y);
  
  double cte = (vector_to_wp_x * wp_angle_y - vector_to_wp_y * wp_angle_x) / len_wp_angle;
  
  wp_angle_x = 0.1;
  wp_angle_y = 0.0;
  double epsi = 0.0;
  if (10 < _ptsx.size())
  {
      wp_angle_x  = _ptsx[10] - _ptsx[0];
      wp_angle_y  = _ptsy[10] - _ptsy[0];
      epsi = psi - atan2(wp_angle_y, wp_angle_x);
    
  }
  */


  cout << "cte: " << cte << "    epsi:" << epsi*180 / 3.14 << endl;

  Eigen::VectorXd state(6);
  state << x, y, psi, v, cte, epsi;


  vector<double> result = mpc_agent.Solve(state, coeffs);


  res.steering = result[0] / deg2rad(25);
  res.throttle = result[1];
  
  vector<double> mpc_x(result.begin()+2, result.begin()+2+N);
  vector<double> mpc_y(result.begin()+2+N, result.end());
  
  res.mpc_x = mpc_x;
  res.mpc_y = mpc_y;
  
  
  return true;
}



// Main
//////////////////////////////////////////////////////////////////
int main(int argc, char **argv) 
{
  ros::init(argc, argv, "mpc_server");
  ros::NodeHandle n;
  
  ros::ServiceServer service = n.advertiseService("mpc_service", mpc_cb);
  ROS_INFO("MPC is ready ...");
  
  ros::spin();
}
