#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <math.h>

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 0;
double dt = 0;
// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

const int HORIZON = 5;
const int GREENHORIZON = 15;
const int TH_MULTI = 5;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {
  step = 0;
}
MPC::~MPC() {}

vector<double> MPC::getSteerThrottle(vector<double> ptsx_, vector<double> ptsy_, double px_, double py_, double psi_, double dpsi_, double v_) {
  ptsx = ptsx_;
  ptsy = ptsy_;

  px = px_;
  py = py_;
  psi = psi_;
  dpsi = dpsi_;
  v = v_;

  buildYellow();


  vector<double> r = getBestDPsiAndThrottle();
  steering = r[0] * TH_MULTI;
  double throttle = r[1];
  
  buildGreen(psi);

  cout << "////// psi \t\t" << psi << endl;
  cout << "////// dPsi \t\t" << dpsi << endl;
  cout << "////// v \t\t" << v << endl;
  cout << "steering \t\t" << steering << endl;
  cout << "throttle \t\t" << throttle << endl;

  vector<double> res = {steering, throttle};

  cout << "----------------------------------------------" << endl;

  return res;
}

void MPC::buildYellow() {
  Eigen::VectorXd ptsy2(6);
  Eigen::VectorXd ptsx2(6);

  for (int i = 0; i < 6; i++) {
    double x = ptsx[i] - px;
    double y = ptsy[i] - py;
    ptsx2[i] = x * cos(-psi) - y * sin(-psi);
    ptsy2[i] = x * sin(-psi) + y * cos(-psi);
  }

  coeffs = polyfit(ptsx2, ptsy2, 3);
  cout << "coeffs " << coeffs << endl; 

  yellowX.clear();
  yellowY.clear();

  for (int i = 0; i < 100; i++) {
    yellowX.push_back(i);
    yellowY.push_back(polyeval(coeffs, i));
  }
}

void MPC::buildGreen(double psi) {

  greenX.clear();
  greenY.clear();

  for (int i = 0; i < GREENHORIZON; i++) {
    greenX.push_back(i);
    vector<double> stateAtDt = getStateAtDt(0, 0, psi, steering, v, i);
    greenY.push_back(stateAtDt[1]);
  }

}

vector<double> MPC::getStateAtDt(double x_, double y_, double psi_, double dpsi_, double v__, double dt) {

  double v_;
  psi_ = M_PI / 2;
  if (isnan(v__) || v__ == 0) v_ = 1;
  else v_ = v__;

  double psiNew = psi_ + Lf/v_ * dpsi_ * dt;

  double xNew = x_ + dt * v_ * cos(psiNew);
  double yNew = y_ + dt * v_ * cos(psiNew);
  double vNew = v_;

  vector<double> res;
  res.push_back(xNew);
  res.push_back(yNew);
  res.push_back(psi_);
  res.push_back(psiNew);
  res.push_back(dpsi_);  
  res.push_back(vNew);

  return res;
}

double MPC::getError(double psi_, double dpsi_) {
  double error = 0;

  for (int dt = 0; dt < HORIZON; dt++) {
    vector<double> state = getStateAtDt(0, 0, psi_, dpsi_, v, dt);

    double x_mpc = state[0];
    double y_mpc = state[1];

    double y_tra = polyeval(coeffs, x_mpc);
    error += (y_mpc - y_tra) * (y_mpc - y_tra);
  }
  return error;
}

vector<double> MPC::getBestDPsiAndThrottle() {
  step++;
  double bestPsi;
  double bestDPsi;
  double bestError = 9999999;

  double thisDpsi = 0;
  for (double thisDpsi = -0.1; thisDpsi < 0.1; thisDpsi += 0.0001) {
        double error = getError(psi, thisDpsi);
        if (error < bestError) {
          bestError = error;
          bestDPsi = thisDpsi;
      }
  }

  vector<double> state = getStateAtDt(0, 0, psi, bestDPsi, v, 1);
  vector<double> res;
  cout << "bestError \t" << bestError << endl;

  double throttle = 0.25;
  if (bestError > 0.25) throttle = 0.15;
  if (bestError > 0.5) throttle = 0.1;
  if (bestError > 1) throttle = 0.05;
  if (bestError > 2) throttle = 0.00;
  if (bestError > 3) throttle = 0.00;

  if (step < 30) throttle = 0.3;

  res.push_back(state[4]);
  res.push_back(throttle);
  return res;
}

vector<double> MPC::getYellowX() {
  return yellowX;
}

vector<double> MPC::getYellowY() {
  return yellowY;
}

vector<double> MPC::getGreenX() {
  return greenX;
}

vector<double> MPC::getGreenY() {
  return greenY;
}

// Evaluate a polynomial.
double MPC::polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd MPC::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
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
