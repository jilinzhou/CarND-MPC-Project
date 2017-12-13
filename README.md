# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

# Overview

This project implements a Model Predictive Controller (MPC) to control a car in Udacity's Term 2 simulator (Project 5).

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
  * Here is the instructions to install them from the source on my Mac (should also work on Linux but not verified).

  ```
  # ipopt install instructions
  wget http://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.tgz
  tar xvzf Ipopt-3.12.7.tgz
  # Download 3rd party dependencies
  cd Ipopt-3.12.7/ThirdParty/Blas
  ./get.Blas
  cd ../Lapack
  ./get.Lapack
  cd ../Mumps
  ./get.Mumps
  cd ../Metis
  ./get.Metis
  # Download MA27 solver from http://www.hsl.rl.ac.uk/ipopt/ and put it into place
  cd ../HSL/
  tar xvzf ~/Downloads/coinhsl-2014.01.10.tar.gz
  mv coinhsl-2014.01.10 coinhsl
  # Configure, build and install
  cd ../..
  mkdir build
  cd build
  ../configure --prefix=/opt/local
  make -j4
  sudo make install
  #Its header files and libs should be installed at /opt/local/include/coin/ and /opt/local/lib/ respectively now.
  ```

  ```
  # cppAD install instructions
  wget https://www.coin-or.org/download/source/CppAD/cppad-20171201.gpl.tgz
  tar xvzf cppad-20171201.gpl.tgz
  cd cppad-20171201
  mkdir build && cd build
  cmake -Dcppad_prefix=/opt/local -Dcmake_install_includedirs=include -Dcppad_postfix=coin -Dcppad_cxx_flags="-Wall -ansi -pedantic-errors -std=c++11 -Wshadow" -Deigin_prefix=/opt/local -Dipopt_prefix=/opt/local ../
  sudo make install
  #Its header files and lib should be installed at /opt/local/include/coin/ and /opt/local/lib/coin/ respectively now.
  ```

* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.

## Basic Build Instructions

Since I am using macport to install all third party libs on my mac, they are installed under /opt/local by default. Therefore, some extra include directories and link directories are added to the original CMakeLists.txt to make build work properly.

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Code Style

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Implementation

### Kinematic model

The kinematic model is directly copied from the course lectures as the following:

```
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```
Where

- `x, y` : position
- `psi` : orientation
- `v` : velocity
- `cte` : cross-track error
- `epsi` : orientation error

These are the state variables of the model. Two control inputs are steering angle (`delta`) and throttle (`a`).

### Polynomial Fitting and MPC Preprocessing

The reference trajectory is sent from the simulator at every iteration as set of 6 points given in their [`x`,`y`] world coordinates. The first to do is to convert them into vehicle's local coordinate frame in which x-axis points to car's travel direction, z-axis points up, and y-axis points to the left of the car to form a RHS coordinate system. A 3rd order polynomial is then estimated from them. A 3rd order polynomial curve is C2 continuous so it has a smooth curvature.   

### Latency

To compensate the 100 ms latency simulated in the control loop, the car's current state is projected forward by the same amount using the kinematic model before feeding it into the MPC solver. Ideally, we hope that the computed control input is applied to the car when it is exactly at the projected state.  

### Parameter Tuning

#### weights for each cost item in objective function

Our objective function has a total of 7 terms. The first three terms reflect the vehicle's state tracking including position, heading, and velocity. The following two terms focus on the minimization of the magnitude of control inputs: steering angle (delta) and throttle (a). The last two terms are added with the goal of avoiding abrupt changes in the control inputs.

To understand the effects of each cost term better, the latency is first set to be zero. The prediction steps (N) and prediction time internal (dt) are fixed at 12 and 100ms respectively. As a starting point, all weights set to be 1.0. Considering they are not normalized, equal weights do not imply each term to have equal importance in the final solution. Three different reference velocities are 20 mph, 40 mph, and 60 mph, are tested. With the reference velocity of 20 mph, the vehicle is able to stay on the track for the whole loop. However, there is not much luck with the other two; the vehicle is off the track right after it reaches the first sharp turn. The discontinuity of consecutive steering angles are quite obvious. These simple tests show that our kinematic model works reasonably well at low speed. Another observation is that the reference velocity can be reached very quick and tracked pretty well. However, this can be problematic during the sharp turns in which the centrifugal force can push the car off the track. In real life, we always slow down during the turn and fast steerings make us feel uncomfortable and the car slid. There are two ways which may solve the issue. One is to generate speed profile in realtime based on the curvature of reference trajectory. The second one is to lower the weight for the velocity tracking term in relative to that of cross error and orientation error tracking. In summary, the general strategy to tune the weights are as the following:

* Add more weights to the last two terms to avoid abrupt changes in control inputs.
* Focus more on the minimization of cross track and orientation errors in relative to velocity tracking errors.
* Put more weights on orientation error than that of cross track error.

After a few trials, the final weights chosen for each cost term are given as the following:

* position tracking error: 30.0
* orientation tracking error: 30.0
* velocity tracking error: 1.0
* magnitude of steering angle: 1.0
* magnitude of throttle: 1.0
* first derivative of steering angle magnitude: 100.0
* first derivative of throttle magnitude: 50.0

By using this weight vector, the car is able to stay in track for the whole loop at a reference velocity up to 60 mph. But it fails once the extra 100 ms latency is added to the control loop. Once again, the noticeable effect is that the gap in between consecutive steering angles are way too big. So I keep increasing the weights for the last two terms till the car can stay in track for the whole loop. The highest reference velocity achieved is 55 mph with the following weights:

* position tracking error: 50.0
* orientation tracking error: 500.0
* velocity tracking error: 1.0
* magnitude of steering angle: 1.0
* magnitude of throttle: 1.0
* first derivative of steering angle magnitude: 300000.0
* first derivative of throttle magnitude: 5000.0

#### Prediction steps (N) and time interval (dt)

I start the tuning of N by reducing it to 6 while keeping time interval and weights as before. Visually, the MPC predicted trajectory does not follow the reference trajectory well during sharp turns due to its half prediction horizon. Sometimes it even bends to another direction. When N is set to be 8, the MPC trajectory follows the reference trajectory much better. In terms of computation time, it takes about 8 ms to solve the optimization problem with N = 8 while it takes about 11 ms if N = 12 at every iteration on my laptop. With N = 8, the car can stay in track with a reference velocity up to 60 mph.

Now we know that a good prediction horizon is about 800 ms (N * dt = 8 * 100ms). We can try different time intervals (dt) to understand their effects. While keeping the weights the same, three other combinations of N and dt are tried: dt = 50 ms with N = 16, dt = 50 ms with N = 8, and dt = 200 ms with N = 4. The first two combinations works as expected but not the last one. For dt = 200 ms, the main reason might be that the slow changing control inputs cannot keep pace with the fast changing system dynamics. The results suggest that the prediction horizon should be bigger than the whole latency in the control loop and a smaller dt improves the accuracy.

In the submitted code, I left N = 8 and dt = 50 ms (N=10 and dt = 100ms might be a better choice). Certainly, the final chosen values are far from optimal and we should be able to reach a higher speed with more tuning.
