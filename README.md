# Autonomous Vehicle
This repository contains the vehicle control proejct for CSE4152 (Programming Languages) at Sogang University, 2023.

## Prerequisites
To install GymSG, please refer to [installGymSG.md].
The versions below are not accurate. However, these versions are recommended.
* Python >= 3.8
* GymSG (using gymSG.zip in this repo)
* matplotlib >= 3.6.3
* numpy >= 1.24.4
* scipy >= 1.10.0
* opencv-python >= 4.7.0.68

## Usage
### Running Autonomous Vehicle
To run the autonomous vehicle, you can execute *autonomous_vehicle.py*.
```
python3 autonomous_vehicle.py
```
![image](https://github.com/user-attachments/assets/14ae02cd-1330-4626-bc08-37464e11f5ab)

This command displays the autonomous driving and graphs of the PID control.

### Testing Lane Detection
To test the lane detection module, you can execute *test_lane_detection.py*.
```
python3 test/test_lane_detection.py
```
After you run the above command, you can drive the car with up, down, left and right keys on your keyboard.

The lane detection is displayed on the Figure 1.

![image](https://github.com/user-attachments/assets/a13278e8-6b49-4dc5-ad7c-9c8b98f95ba8)

### Testing Waypoint Prediction
To test the waypoint prediction module, you can execute *test_waypoint_prediction.py*.
```
python3 test/test_waypoint_prediction.py 
```
After you run the above command, you can drive the car with up, down, left and right keys on your keyboard.

The predicted waypoint is displayed on the Figure 1.

![image](https://github.com/user-attachments/assets/46fee002-4c0a-429b-a531-dce9985cb3c3)



## Project 
![image](https://github.com/user-attachments/assets/406cee66-d332-4e85-a2f9-ee376ba65e29)

* Goal: implement a modular pipeline framework which controls vehicle autonomously.
* Simulator: OpenAIGYM
* CarRacing information
  * Action: steering, acceleration, brake
  * Sensor input: 96x96x3 screen

### Lane Detection
![image](https://github.com/user-attachments/assets/69e230d3-6c26-41d2-b256-260b68ca7dbe)

By convolving an image with edge filters, two directional gradient maps are obtained.

![image](https://github.com/user-attachments/assets/7acafe21-cd20-41c0-b525-7868f74997af)

In order to navigate the car, B(Basis)-Spline Curve is used. It fits detected mark pixels to a more semantically meaningful curve model. *scipy.interpolate.splprep* is used for fitting.


### Path Planning
* Road Center
  * Use the lane boundary splines and derive lane boundary points for 6 equidistant spline parameter values using *scipy.interpolate.splev*.
  * Determine the center between lane boundary points with the same spline parameter.
    
* Path Smoothing
  * Improve the path by minimizing the following objective regarding the waypoints xgiven the center waypoints y.
    ![image](https://github.com/user-attachments/assets/554df5e6-bfd0-4182-bfc2-5b686f5b36bc)

* Target Speed Prediction
  * Implement a function that outputs the target speed for the predicted path in the state image, using
    ![image](https://github.com/user-attachments/assets/392b125d-16da-4665-9eca-445f52ecd991)

### Control
![image](https://github.com/user-attachments/assets/bb923664-0b6c-4354-ab36-14d7d633e8f4)

Proportional-Integral-Derivative (PID) Control is used for longtitudal control.

![image](https://github.com/user-attachments/assets/96b4b791-89fc-4233-91d7-b26f6e0f6927)

For lateral control, Stanley Control was used. It combines heading (first term) and cross-track (second term) error.



[installGymSG.md]: https://github.com/juhynl/autonomous-vehicle/blob/main/installGymSG.md
