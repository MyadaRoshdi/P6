# Extended Kalman Filter for object tracking 
Self-Driving Car Engineer Nanodegree Program

In this project I used Extended kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Obtaining RMSE values for estimated object location( px, py) and velocity ( vx, vy) output coordinates  <= [.11, .11, 0.52, 0.52] when using the file: "obj_pose-laser-radar-synthetic-input.txt" which is the same data file the simulator uses for Dataset 1. 

[//]: # (Image References)

[image0]: ./imgs/code_simulator.jpg "code_simulator"
[image1]: ./imgs/KF_concept.jpg "KF_concept"
[image2]: ./imgs/EKF_flowchart.jpg "EKF_flowchart"

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF



## Comunication between the code and simulator

![code_simulator][image0]

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `


## Code Style

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Kalman filter main concept

Generally, Kalman filter is running iteratively 2- basic steps, **State Predict** & **Measurment update** for each sensor measurment as shown below:

![KF_concept][image1]

## Extended Kalman filter Algorithm flow

In Extended Kalman filter (Sensor fusion), here I use the following:

1.) Data from both Lidar and Radar

2.) State prediction is done using **Bayes Rule** for both sensors

3.) Measurment update is using linear motion model for Lidar and non-linear for Radar( using Taylor series i.e. jacobian transformation step)

**Note:** Laser provides high accuracy position info (px,py) whilt Radar provides precise velocity info (vx,vy) using doppler effect.

**Note:** Laser measurments are provided in Cartisian co-ordinates while Radar in Polar co-ordinates, so in state predicition step, we always need to convert form Polar to Cartisian in Radar case to set the state vector x=[px, py, vx, vy]/
Here's the flowchart for the used algorithm for 

![EKF_flowchart][image2]
