# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

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
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## The Model

### State [x, y, psi, v]

1. Position (x,y) in 2D
2. Orientation 'psi'
3. Velocity 'v'
4. Cross-Track Error 'cte'
5. Orientation Error 'epsi'

### Actuators

1. Steering Wheel 'delta'
2. Throttle and Break peddle 'a' (positive or negative acceleration)

### Update Equations

x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> * cos(psi<sub>t</sub>) * dt

y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub> * sin(psi<sub>t</sub>) * dt

psi<sub>t+1</sub> = psi<sub>t</sub> + (v<sub>t</sub>/L<sub>f</sub>) * delta<sub>t</sub> * dt
        
v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub>00 k* dt

cte<sub>t+1</sub> = f(x<sub>t</sub>) - y<sub>t</sub> + v<sub>t</sub> * sin(epsi<sub>t</sub>) * dt;

epsi<sub>t+1</sub> = psi<sub>t</sub> - psi_des + v<sub>t</sub>/L<sub>f</sub> * delta<sub>t</sub> * dt;

where L<sub>f</sub> is the distance between the front of the vehicle and its center of gravity. The larger the vehicle, the slower the turn rate.

## Timestep Length and Elapsed Duration (N & dt)

After trial and error, the following values were chosen for the hyper parameters in the project,

* Prediction Horizon (T) = N * dt = 1s

First T is chosen such that it is in reasonable range, few seconds. Beyond horizon, since the environment changes enough, it won't make sense to predict further into future. Then N and dt are tuned accordingly

* Timestep Length (N) = 10

Higher the value of N, the vehicle overshoots reference trajectory and goes off the track and also takes longer to compute.

* Elapsed Duration (dt) = 0.1s

dt should be as small as possible

## Polynomial Fitting and MPC Preprocessing

* Transform points into vehicle co-ordinate system by subtracting each point from the current position of the vehicle

* Transform orientation to 0 so that the vehicle is heading straight forward by rotating each point to psi degree

* Convert vector of points to Eigen vector

* Use polyfit function to fit the points to a 3rd degree polynomial

* Use polyeval function to evaluate the polynomial for calculating cross-track error

## Model Predictive Control with Latency

An Actuation command won't excite instantly. There will be delay as the command propagates through the system. Also contributing factor to the latency is actuator dynamics. For example the time elapsed between when you command a steering angle to when that angle is actually achieved. In this project I try adding penalty for speed and steer (line 63) so that the vehicle slows down at turns giving more time for computation.
