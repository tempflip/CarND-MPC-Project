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
MPC::MPC() {}
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

  /*
  cout << "PX \t\t" << px << endl;
  cout << "PY \t\t" << py << endl;
  cout << "PSI \t\t" << psi << endl;

  cout << "ptsx:\t\t";
  for (int i = 0; i < 6; i++) {
    cout << ptsx[i] << "\t";
  }
  cout << endl;

  cout << "ptsy:\t\t";
  for (int i = 0; i < 6; i++) {
    cout << ptsy[i] << "\t";
  }
  cout << endl;
  */

  double bestPsi = getBestDPsi();
  buildGreen(bestPsi);


  double steering = (bestPsi - M_PI/2) / M_PI/2;
  //double steering = bestPsi;
  cout << "////// dPsi \t\t" << dpsi << endl;
  cout << "bestPsi \t\t" << bestPsi << endl;
  cout << "steering \t\t" << steering << endl;

  vector<double> res = {steering, 0.3};

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

  for (int i = 0; i < 30; i++) {
    greenX.push_back(i);
    vector<double> stateAtDt = getStateAtDt(0, 0, psi, dpsi, v, i);
    greenY.push_back(stateAtDt[1]);
  }

}

vector<double> MPC::getStateAtDt(double x_, double y_, double psi_, double dpsi_, double v__, double dt) {

  double v_;
  if (isnan(v__) || v__ == 0) v_ = 1;
  else v_ = v__;

  cout << "############### v_" << v_ << endl;

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

  for (int dt = 0; dt < 30; dt++) {
    vector<double> state = getStateAtDt(0, 0, psi_, dpsi_, v, dt);

    double x_mpc = state[0];
    double y_mpc = state[1];

    double y_tra = polyeval(coeffs, x_mpc);
    error += (y_mpc - y_tra) * (y_mpc - y_tra);
  }
  return error;
}

double MPC::getBestDPsi() {
  double dpsi_ = dpsi;

  double bestPsi;
  double bestDPsi;
  double bestError = 9999999;
  for (int i = 1; i < 10; i ++) {

      double degree = M_PI / i;
      double error = getError(degree, dpsi_);

      if (error < bestError) {
        bestError = error;
        bestPsi = degree;
        cout << "Best error: \t\t" << degree << "\t" << error << endl;
    }
  }


  vector<double> state = getStateAtDt(0, 0, bestPsi, dpsi_, v, 1);

  return state[3];
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

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = 0;
  // TODO: Set the number of constraints
  size_t n_constraints = 0;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  return {};
}
