//======================================================================================================================
#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
//======================================================================================================================
using CppAD::AD;
//======================================================================================================================
size_t N = 15;
double dt = 0.10;
const double Lf = 2.67;
//======================================================================================================================
const int idx_x = 0;
const int idx_y = idx_x + N;
const int idx_psi = idx_y + N;
const int idx_v = idx_psi + N;
const int idx_cte = idx_v + N;
const int idx_epsi = idx_cte + N;
const int idx_delta = idx_epsi + N;
const int idx_a = idx_delta + N - 1;
//======================================================================================================================

class FG_eval
{
public:
  using ADvector = CPPAD_TESTVECTOR(AD<double>);

 public:
  // Fitted polynomial coefficients
  FG_eval(Eigen::VectorXd coeffs)
  {
    coeffs_ = coeffs;
  }

  void operator()(ADvector& fg, const ADvector& vars)
  {
    const double ref_cte = 0;
    const double ref_epsi = 0;
    const double ref_v = 40;

    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (int i = 0; i < N; i++)
    {
      fg[0] += CppAD::pow(vars[idx_cte + i] - ref_cte, 2);
      fg[0] += CppAD::pow(vars[idx_epsi + i] - ref_epsi, 2);
      fg[0] += CppAD::pow(vars[idx_v + i] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int i = 0; i < N - 1; i++)
    {
      const double kSteeringPenaltyFactor = 125;
      // The following value could be used for ref_v = 80:
      // const double kSteeringPenaltyFactor = 50000;
      // The following value could be used for ref_v = 100:
      //const double kSteeringPenaltyFactor = 100000;
      fg[0] += kSteeringPenaltyFactor * CppAD::pow(vars[idx_delta + i], 2);
      fg[0] += 1.0 * CppAD::pow(vars[idx_a + i], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int i = 0; i < N - 2; i++)
    {
      fg[0] += CppAD::pow(vars[idx_delta + i + 1] - vars[idx_delta + i], 2);
      fg[0] += CppAD::pow(vars[idx_a + i + 1] - vars[idx_a + i], 2);
    }

    // Initial constraints
    fg[1 + idx_x] = vars[idx_x];
    fg[1 + idx_y] = vars[idx_y];
    fg[1 + idx_psi] = vars[idx_psi];
    fg[1 + idx_v] = vars[idx_v];
    fg[1 + idx_cte] = vars[idx_cte];
    fg[1 + idx_epsi] = vars[idx_epsi];

    // The rest of the constraints
    for (int i = 0; i < N - 1; i++) 
    {
      // The state at time t+1 .
      const auto x1 = vars[idx_x + i + 1];
      const auto y1 = vars[idx_y + i + 1];
      const auto psi1 = vars[idx_psi + i + 1];
      const auto v1 = vars[idx_v + i + 1];
      const auto cte1 = vars[idx_cte + i + 1];
      const auto epsi1 = vars[idx_epsi + i + 1];

      // The state at time t.
      const auto x0 = vars[idx_x + i];
      const auto y0 = vars[idx_y + i];
      const auto psi0 = vars[idx_psi + i];
      const auto v0 = vars[idx_v + i];
      const auto cte0 = vars[idx_cte + i];
      const auto epsi0 = vars[idx_epsi + i];

      // Only consider the actuation at time t.
      const auto delta0 = vars[idx_delta + i];
      const auto a0 = vars[idx_a + i];

      const auto f0 = coeffs_[0] + coeffs_[1] * x0 + coeffs_[2] * CppAD::pow(x0, 2) + coeffs_[3] * CppAD::pow(x0, 3);
      const auto psides0 = CppAD::atan(coeffs_[1] + 2*coeffs_[2]*x0 + 3*coeffs_[3]*CppAD::pow(x0, 2));

      fg[2 + idx_x + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + idx_y + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + idx_psi + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[2 + idx_v + i] = v1 - (v0 + a0 * dt);
      fg[2 + idx_cte + i] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + idx_epsi + i] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }

private:
  Eigen::VectorXd coeffs_;
};


//======================================================================================================================
// MPC class definition implementation.

MPC::MPC() {}
MPC::~MPC() {}

//----------------------------------------------------------------------------------------------------------------------

std::vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
  bool ok = true;
  size_t i;
  using Dvector = CPPAD_TESTVECTOR(double);

  const double px = state[0];
  const double py = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  const int kDimState = 6;
  const int kDimActuators = 2;
  const int kNumVars = kDimState * N + kDimActuators * (N-1);
  // TODO: Set the number of constraints
  const int kNumConstraints = kDimState * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(kNumVars);
  for (int i = 0; i < kNumVars; i++)
  {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(kNumVars);
  Dvector vars_upperbound(kNumVars);
  for (int i = 0; i < idx_delta; i++)
  {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  const double kDeltaMax = 15 * M_PI / 180.0;
  for (int i = idx_delta; i < idx_a; i++)
  {
    vars_lowerbound[i] = -kDeltaMax;
    vars_upperbound[i] = kDeltaMax;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = idx_a; i < kNumVars; i++)
  {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(kNumConstraints);
  Dvector constraints_upperbound(kNumConstraints);
  for (int i = 0; i < kNumConstraints; i++)
  {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[idx_x] = px;
  constraints_lowerbound[idx_y] = py;
  constraints_lowerbound[idx_psi] = psi;
  constraints_lowerbound[idx_v] = v;
  constraints_lowerbound[idx_cte] = cte;
  constraints_lowerbound[idx_epsi] = epsi;

  constraints_upperbound[idx_x] = px;
  constraints_upperbound[idx_y] = py;
  constraints_upperbound[idx_psi] = psi;
  constraints_upperbound[idx_v] = v;
  constraints_upperbound[idx_cte] = cte;
  constraints_upperbound[idx_epsi] = epsi;


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

  std::vector<double> result;
  double delta = 0.0;
  double a = 0.0;

  int n_mean = 3;
  for (int i = 0; i < n_mean; i++)
  {
    delta += solution.x[idx_delta + i] / n_mean;
    a += solution.x[idx_a + i] / n_mean;
  }

  result.push_back(delta);
  result.push_back(a);

  for (i = 0; i < N; ++i)
  {
    result.push_back(solution.x[idx_x + i]);
    result.push_back(solution.x[idx_y + i]);
  }

  return result;
}

//======================================================================================================================
