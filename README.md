# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

Model Predictive Control reframes the task of following a trajectory as an optimization problem. The solution to the optimization problem is the optimal trajectory. MPC involves simulating different actuator inputs, predicting the resulting trajectory and selecting the trajectory with the minimum cost.

In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates through the system. A realistic delay might be on the order of 100 milliseconds. This is a problem called "latency", and it's a difficult challenge for some controllers - like a PID controller - to overcome. But a Model Predictive Controller can adapt quite well because we can model this latency in the system.

##### The Model:

The current implementation uses Kinematic model based equations as shown in the image below.

![Alt text](images/model.png | width=100)

Lf measures the distance between the front of the vehicle and its center of gravity. The larger the vehicle, the slower the turn rate.

Control inputs are: [δ,a]

##### Timestep Length and Elapsed Duration (N & dt):

The prediction horizon is the duration over which future predictions are made. We’ll refer to this as T.

T is the product of two other variables, N and dt.

+ N is the number of timesteps in the horizon.
+ dt is how much time elapses between actuations. 

For example, if N were 20 and dt were 0.5, then T would be 10 seconds.

N, dt, and T are hyperparameters those need to be tuned. T should be as large as possible, while dt should be as small as possible.

The goal of Model Predictive Control is to optimize the control inputs: [δ,a]. The optimizer will tune these inputs until a low cost vector of control inputs is found. The length of this vector is determined by N. Thus N determines the number of variables optimized by the MPC. This is also the major driver of computational cost.

MPC attempts to approximate a continuous reference trajectory by means of discrete paths between actuations. Larger values of dt result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory. This is sometimes called "discretization error".


##### Polynomial Fitting and MPC Preprocessing:

Third Degree polynomial is fitted to the waypoints provided by simulator after transforming them to car coordinate system. The polynomial coefficients are then used to calculate cte and epsi. These are used by solver as well to create ref trajectory.

##### Model Predictive Control with Latency:

We compute the state with the delay (100ms) factored in using our kinematic model before feeding it to the object that will solve for what we should do next.


The Unity Simulator used for this project can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).


##### Simulator Output Recording

[![MPC Control](https://img.youtube.com/vi/n34g0S3yf0c/0.jpg)](https://www.youtube.com/watch?v=n34g0S3yf0c "MPC Control")



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
 
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
       
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

