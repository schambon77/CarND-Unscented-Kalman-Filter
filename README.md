# Unscented Kalman Filter Project 
Self-Driving Car Engineer Nanodegree Program

## Project Ojectives
In this project, an Unscented Kalman Filter is utilized to estimate the state of a moving object of interest with noisy lidar and radar measurements. The goal is to obtain RMSE values that are lower than the tolerance outlined in the project rubric. 

## Project Files
* [Source code](https://github.com/schambon77/CarND-Unscented-Kalman-Filter/tree/master/src)
* [ReadMe](https://github.com/schambon77/CarND-Unscented-Kalman-Filter/blob/master/README.md)

## Project Rubric Points
The project rubric points can be found [here](https://review.udacity.com/#!/rubrics/783/view).

### Compiling

The [source code](https://github.com/schambon77/CarND-Unscented-Kalman-Filter/tree/master/src) compiles without errors with `cmake` and `make` by executing the following commands from the repository top folder:

mkdir build
cd build
cmake ..
make
./UnscentedKF

### Accuracy

When testing with the simulator, the px, py, vx, vy output coordinates have a final RMSE that meets the success criteria of <= [.09, .10, .40, .30].

### Follows the Correct Algorithm

The unscented Kalman filter algorithm follows the general processing flow as taught in the preceding lessons.

The `UKF` constructor finishes the initialization of several member variables, including the creation of both `Xsig_pred_` and `weights_` matrices.

The process noise standard deviation longitudinal acceleration `std_a_` is set to 1. The process noise standard deviation yaw acceleration `std_yawdd_` is set to 1.5.


The UKF::ProcessMeasurement() method handles the first measurements appropriately, whether it is a laser or radar measurements, in order to initialize the state vector x and covariance matrice P. In the case of a radar measurement, the first 2 diagonal elements of P are initialized using the standard deviation of the radius `std_radr_` as follows: `std_radr_*std_radr_/2`. In the case of a laser measurement, standard deviations of px and py are used. Other terms are initialized to 1.

The unscented Kalman filter algorithm carries on to first predict, then update.

Upon receiving a measurement after the first, the algorithm predicts object position to the current timestamp by:
* generating sigma points,
* predicting the sigma points,
* and using the predicted sigma points to predict the state.

The algorithm then updates the prediction using the new measurement. It can handle both radar and lidar measurements, although they can both be turned off by the use of respecive booleans `use_laser_` and `use_radar_`. The algorithm does so by:
* predicting the measurement,
* and updating the state.

The correct measurement function for the given sensor type (UKF::UpdateLidar() for laser measurements, UKF::UpdateRadar() for radar measurements).

### Code Efficiency

The algorithm tries to avoid unnecessary calculations. The prescribed code style is respected as much as possible. The source code was edited with the Eclipse IDE for C/C++ Developers (Oxygen).

### Suggestions to Make Your Project Stand Out!

RMSE results for different configurations:

|Configuration|X|Y|VX|VY|
|:--------:|:---:|:---:|:---:|:---:| 
|UKF both sensors| 0.0687 | 0.0834 | 0.2920 | 0.2526 |
|UKF laser only| 0.1024 | 0.0985 | 0.4642 | 0.2795 |
|UKF radar only| 0.1517 | 0.2168 | 0.4363 | 0.2586 |
|EKF both sensors| 0.0964 | 0.0853 | 0.4154 | 0.4316 |
