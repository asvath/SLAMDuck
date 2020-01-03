# SLAMDuck 
We provide a brief description of our project (Full report will be uploaded after grading has been completed). For details on the code, please see the Code section.

## Introduction
The goal of our project is to explore the recursive Extended Kalman Filter (EKF) SLAM algorithm. In order to achieve this goal, we draw on our motivation to acquire hands-on robotics experience. We obtained a robotics starter kit from the Duckietown foundation (https://www.duckietown.org/). We utilized the starter kit to build a robot (Duckiebot) and a toy city
(Duckietown). The starter kit contains several “off-the-self” components including a monocular camera and a Raspberry Pi3
for the Duckiebot; and driveable surfaces, lane markings and traffic signs with AprilTags for the Duckietown. 

## Problem Statement
The problem of SLAMDuck is: “to come up with an estimate of the true pose, (x,y,theta), of the Duckiebot at timesteps k1:k2 and the estimate of the true positions (x,y) of the N static landmarks, given a sequence of the Duckiebot’s inputs and measurements”.
The inputs are the wheel velocities, landmarks are AprilTags and measurements are the range and bearing measurements from the Duckiebot’s camera to the landmarks. Range and bearing measurements are extracted from detection of the AprilTags in the camera images.

## Theory
### Definition of Frames
To tackle the problem of SLAMDuck, we define the following frames:
* World Frame, Fw: global frame. Estimates of the Duckiebot’s pose and the landmarks positions are made relative Fw. The origin of Fw is set at the start position of the Duckiebot.
* Duckie frame, FD: frame that is attached to center between the left and right wheels of the Duckiebot. The x-axis is in the direction of the heading of the Duckiebot
* Camera frame, Fc: frame that is attached to the Duckiebot’s camera and distance, d, away from FD. The orientation of Fc is the same as FD. All measurement of landmark AprilTags are made relative to Fc

![alt text](https://github.com/asvath/SLAMDuck/blob/master/pix/frames.PNG)
