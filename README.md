# Model Predictive Control Project

In this project I created a Model Predictive Controller (MPC) to simulate different actuator inputs, predict the resulting trajectory, and select the trajectory with a minimum cost. The enables the car to turn in the most efficient and safe way.

***

### Project Writeup

#### The Model - Student describes their model in detail. This includes the state, actuators and update equations.

My model loops through the process of combining the actuation & state from the previous timestep to calculate the state for the current timestep. It does this over and over to correct the cross-track error and psi error. The model uses the vehicles **x** and **y** coordinates, the **orientation angle**, and the **velocity** to do this. These are the equations used:

![alt tag](https://github.com/CodyNicholson/Self-Driving_Car_Nanodegree/blob/master/26_Model_Predictive_Control/imgs/mpc.PNG)

#### Timestep Length and Elapsed Duration (N & dt) - Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

I had some trouble choosing my values for **N** and **dt**. I tried guessing at it for a while, but every time I thought I was getting closer to correct values something would go wrong. After reading through the forums and watching the project help video on the Udacity YouTube channel, I was encouraged to use the value 10 for **N** and 0.1 for **dt**. Given these values, the optimizer will have a 1 second duration to determine the correct trajectory for the car to use.

#### Polynomial Fitting and MPC Preprocessing - A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

I preprocessed the waypoints by transforming them to the vehicle's perspective. This  simplifies the process of fitting a polynomial to the waypoints since the vehicle's **x** and **y** coordinates are now at the origin, and the **orientation angle** is zero. You can see the code I used to do this below:

```c++
// Transform waypoints to be from vehicle perspective
for (int i = 0; i < ptsx.size(); i++) {
  double d_x = ptsx[i] - px;
  double d_y = ptsy[i] - py;
  waypoints_x.push_back(d_x * cos(-psi) - d_y * sin(-psi));
  waypoints_y.push_back(d_x * sin(-psi) + d_y * cos(-psi));
}
```

#### Model Predictive Control with Latency - The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates through the system. A realistic delay might be on the order of 100 milliseconds.

This is a problem called latency, and it's a difficult challenge for some controllers - like a PID controller - to overcome. But a Model Predictive Controller can adapt quite well because we can model this latency in the system.

A contributing factor to latency is actuator dynamics. For example the time elapsed between when you command a steering angle to when that angle is actually achieved. This could easily be modeled by a simple dynamic system and incorporated into the vehicle model. One approach would be running a simulation using the vehicle model starting from the current state for the duration of the latency. The resulting state from the simulation is the new initial state for MPC.

My algorithm evaluates the vehicle's state 100ms into the future before calling the MPC solver function. Since the state variables are computed in vehicle coordinate system, the kinematic equations could be used to estimate the state of the vehicle 100ms in the future. This can be done using the update equations and model error equations described in the lessons. In vehicle coordinates, the vehicle is at the origin and so **px**, **py** and **psi** are considered as zero and substituting these values into kinematic equations with **dt**=0.1 will give the state and errors 100ms into the future.

***

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
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
