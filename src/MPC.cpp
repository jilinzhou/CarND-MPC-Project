#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <array>

using CppAD::AD;

// Set the timestep length and duration
size_t N = 8;

double dt = 0.05;

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

// Ideally we hope to achieve zero cross track error and zero orientation error
// and maintain the vehicle at a comfortable speed.
// If we know how vehicle dynamics is simulated inside the simulator,
// we may generate a reference speed profile along the track instead of using a
// fixed reference speed.
// 1 mph = 0.44704 m/s
double ref_v = 60.0 * 0.44704;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
// state vector: [x, y, psi, v, cte, epsi]
// actuator vector: [delta, a]
size_t x_start     = 0;
size_t y_start     = x_start + N;
size_t psi_start   = y_start + N;
size_t v_start     = psi_start + N;
size_t cte_start   = v_start + N;
size_t epsi_start  = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start     = delta_start + N - 1;

class FG_eval {
public:

  typedef CPPAD_TESTVECTOR (AD<double>) ADvector;

  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;

  FG_eval(Eigen::VectorXd coeffs) {
    this->coeffs = coeffs;
  }

  void operator()(ADvector& fg, const ADvector& vars) {
    //
    // Compute objective function first given current guess of solution
    //
    // The cost is stored is the first element of 'fg'
    fg[0] = 0.0;

    // The part of the cost based on the reference state
    for (int t = 0; t < N; t++) {
      // cross track error cost
      fg[0] += 50.0 * CppAD::pow(vars[cte_start + t], 2);

      // orientation error cost
      fg[0] += 500.0 * CppAD::pow(vars[epsi_start + t], 2);

      // speed error cost
      fg[0] += 1.0 * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators
    // N steps --> N-1 actuation commands
    for (int t = 0; t < N - 1; t++) {
      // steering commands
      fg[0] += 1.0 * CppAD::pow(vars[delta_start + t], 2);

      // throttle commands
      fg[0] += 1.0 * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations
    // to avoid abrupt changes.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 300000.0 *
               CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 5000.0 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    //
    // Setup Constraints
    //

    // Initial Constraints as a boundary condition for the solver
    fg[1 + x_start]    = vars[x_start];
    fg[1 + y_start]    = vars[y_start];
    fg[1 + psi_start]  = vars[psi_start];
    fg[1 + v_start]    = vars[v_start];
    fg[1 + cte_start]  = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the contraints
    // Let's assume that we have a reasonable vehicle kinematic model. Given
    // vehicle's state at t_k, we can predict its state
    // at t_k+1 by using this kinematic model. Therefore, we can use this as the
    // contraints to solve our minization problem which is to minimize the
    // objective function (fg[0]) as shown above.
    for (int t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1    = vars[x_start + t];
      AD<double> y1    = vars[y_start + t];
      AD<double> psi1  = vars[psi_start + t];
      AD<double> v1    = vars[v_start + t];
      AD<double> cte1  = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0    = vars[x_start + t - 1];
      AD<double> y0    = vars[y_start + t - 1];
      AD<double> psi0  = vars[psi_start + t - 1];
      AD<double> v0    = vars[v_start + t - 1];
      AD<double> cte0  = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0     = vars[a_start + t - 1];

      // from reference trajectory directly
      AD<double> f0 = ((coeffs[3] * x0 + coeffs[2]) * x0 + coeffs[1]) * x0  +
                      coeffs[0];

      // use the first derivative of the reference trajectory
      AD<double> psides0 = CppAD::atan(
        (3 * coeffs[3] * x0 + 2 * coeffs[2]) * x0 + coeffs[1]);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[1 + x_start + t]   = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t]   = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t]   = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
        cte1 - (f0 - y0 + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
        epsi1 - (psi0 - psides0 + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {
  N  = 8;
  Lf = 2.67;
}

MPC::~MPC() {}

vector<double>MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;

  typedef CPPAD_TESTVECTOR (double) Dvector;

  // Number of independent varialbes = 6
  // [x, y, psi, v, cte, epsi]
  size_t n_states = 6;

  // N timesteps ==> N -1 actuations
  size_t n_vars = N * n_states + (N - 1) * 2;

  // Number of constraints
  // 1. n_states (initial state as boundary contraint)
  // 2. (N-1) * n_states (kinematic contraints after each actuation)
  size_t n_constraints = N * n_states;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);

  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  // Set the initial variable values
  vars[x_start]    = state[0];
  vars[y_start]    = state[1];
  vars[psi_start]  = state[2];
  vars[v_start]    = state[3];
  vars[cte_start]  = state[4];
  vars[epsi_start] = state[5];

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // set all non-actuators upper and lower limits.
  // It includes [x, y, psi, v, cte, epsi]
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0.0;
    constraints_upperbound[i] = 0.0;
  }
  constraints_lowerbound[x_start]    = state[0];
  constraints_lowerbound[y_start]    = state[1];
  constraints_lowerbound[psi_start]  = state[2];
  constraints_lowerbound[v_start]    = state[3];
  constraints_lowerbound[cte_start]  = state[4];
  constraints_lowerbound[epsi_start] = state[5];

  constraints_upperbound[x_start]    = state[0];
  constraints_upperbound[y_start]    = state[1];
  constraints_upperbound[psi_start]  = state[2];
  constraints_upperbound[v_start]    = state[3];
  constraints_upperbound[cte_start]  = state[4];
  constraints_upperbound[epsi_start] = state[5];

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

  // std::cout << "Cost " << cost << std::endl;

  std::vector<double> results;

  // current cost and control inputs
  results.push_back(solution.obj_value);
  results.push_back(solution.x[delta_start]);
  results.push_back(solution.x[a_start]);

  // predicted trajectory
  for (int t = 0; t < N; t++) {
    results.push_back(solution.x[x_start + t]);
  }

  for (int t = 0; t < N; t++) {
    results.push_back(solution.x[y_start + t]);
  }
  return results;
}
