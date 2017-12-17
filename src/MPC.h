#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  double px;
  double py;
  double psi;
  double dpsi;
  double v;
  double steering;
  std::vector<double> ptsx;
  std::vector<double> ptsy;
  Eigen::VectorXd coeffs;
  std::vector<double> yellowX;
  std::vector<double> yellowY;
  std::vector<double> greenX;
  std::vector<double> greenY;

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  vector<double> getSteerThrottle(vector<double> ptsx, vector<double> ptsy, double px, double py, double psi, double dPsi, double v);

  vector<double> getYellowX();

  vector<double> getYellowY();

  vector<double> getGreenX();

  vector<double> getGreenY();

  vector<double> getStateAtDt(double x_, double y_, double psi_, double dpsi_, double v_, double dt);

  void buildYellow();

  void buildGreen(double psi);

  double polyeval(Eigen::VectorXd coeffs, double x);

  Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

  double getError(double psi_, double dpsi_);

  double getBestDPsi();
};

#endif /* MPC_H */
