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
  std::vector<double> ptsx;
  std::vector<double> ptsy;
  Eigen::VectorXd coeffs;

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  vector<double> getSteerThrottle(vector<double> ptsx, vector<double> ptsy, double px, double py, double psi);

  vector<double> getPredictedX();

  vector<double> getPredictedY();

  vector<double> getTrajectoryX();
  
  vector<double> getTrajectoryY();

  vector<double> getTrajectoryCarCoordsX();

  vector<double> getTrajectoryCarCoordsY();

  double polyeval(Eigen::VectorXd coeffs, double x);

  Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);


};

#endif /* MPC_H */
