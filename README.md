# SLAMDuck 
We provide a brief description of our project. For details on the code, please see the Data acquision and Code sections below. We also provide our acquired data. This includes the data from our duckiebot and the corresponding vicon data (ground truth). The files will be uploaded soon. 

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

<img src="https://github.com/asvath/SLAMDuck/blob/master/pix/frames.PNG" width="500" height="300"> <img src="https://github.com/asvath/SLAMDuck/blob/master/pix/frames_setup.PNG" width="500" height="500">

### Motion Model
The Duckiebot is a differential drive robot. Hence, using a differential drive model, we determine the translational, vk, and rotational, wk, velocities of the Duckiebot. Next, we define xk to be a 3 + 2N state vector consisting of the Duckiebot’s pose (x,y,theta), and each landmark’s position (lx,ly) at timestep, k; where l denotes the lth landmark. All the parameters in the state vector are relative to Fw. The motion model is then governed by:

<img src="https://github.com/asvath/SLAMDuck/blob/master/pix/motion.PNG" width="400" height="200">

### Measurement Model
The measurements comprise of the range and bearing measurement of the landmark AprilTags relative to Fc. These measurements were extracted from the detection of the AprilTags in the camera images (discussed under code section). The observation model for the lth
landmark at time step k is governed by:

<img src="https://github.com/asvath/SLAMDuck/blob/master/pix/measurement.PNG" width="400" height="100">

### SLAMDuck Algorithm Theory
Our SLAMDuck algorithm then follows the EKF SLAM algorithm described in S. Thrun, W. Burgard, and D. Fox Probabalistic Robotics (2005). chapter 10 page 314. Where our motion model is used in the prediction step and our measurement model is used in the
correction step. Our algorithm outputs the estimate of the true pose of the Duckiebot at timesteps k1:k2 and the estimate of the
true positions of the N static landmarks with respect to Fw.

## Method
### Setup
We followed all setup, including calibration steps as described Duckiebot operationmanual available at:
https://docs.duckietown.org/daffy/opmanual_duckiebot/out/index.html
Our Duckietown was built to according to the specifications described in the Duckietown operation manual available at
https://docs.duckietown.org/daffy/opmanual_duckietown/out/dt_ops_preliminaries.html.

Our final setup is as shown:

<img src="https://github.com/asvath/SLAMDuck/blob/master/pix/top_view_duckietown.jpg" width="500" height="400">

We included cardboard buildings, trees, a pool and 5 toy ducks, as shown purely for aesthetic and entertainment purposes.

The video below shows our experimental setup:
[![Alt text](https://github.com/asvath/SLAMDuck/blob/master/pix/setup.PNG)](https://www.youtube.com/watch?v=59ch7SB2_Eg)


### Data Acquision and preprocessing 
We drove our Duckiebot around Duckietown for two loops in an anti-clockwise direction with start position as depicted in the figure above. The drive was done manually by utilizing the virtual joystick through the Duckietown shell. We logged the data at nominal rate of 30 Hz. The logged data comes in a rosbag (.bag). format containing the left and right velocities of the wheels and the images captured. As the wheel velocities and the images were acquired at different timestamps, we linearly interpolated the wheel velocities at the image observations timestamps. As part of preprocessing, the images were undistorted using the intrinsic camera calibration matrix and the distortion coefficients acquired during camera calibration. We then used the publicly available AprilTags detection library apriltags3
https://github.com/duckietown/apriltags3-py to detect the landmark AprilTags in our images. The detection outputs the relative position of the tags with respect to the Duckiebot’s camera. Using the relative position of the tags we then calculated the range and bearing from the camera to the tags. The 15-degree tilt of our camera was taken into account during the calculations. The range and bearing information were then stored in text files to be used by the SLAMDuck algorithm during the correction step. Please see the code section for all code used to perform the steps described in this section

### Vicon Ground Truth 
To evaluate our algorithm, we acquired ground truth by using the vicon motion capture system available at the Dynamics Systems Laboratory (DSL) at the University of Toronto Institute for Aerospace Studies. The vicon motion capture system is
accurate to within a few millimeters. Our experiment was set up at DSL. Reflective markers were attached to our Duckiebot and its pose was tracked at 300 Hz as it navigated Duckietown. In order to compare the pose estimate of our Duckiebot to the acquired ground truth data, we linearly interpolated the vicon data to the image observations timestamp. Reflective markers were also on our landmark
AprilTag as shown in, The positions of the static landmark AprilTags were acquired by vicon for 30 seconds and the results
were averaged.

<img src="https://github.com/asvath/SLAMDuck/blob/master/pix/reflective_marker_robot.jpg" width="300" height="450"> <img src="https://github.com/asvath/SLAMDuck/blob/master/pix/reflective_marker_landmark.jpg" width="300" height="450"> 

## Results 
### Wheel odometry
We drove our Duckiebot around Duckietown for two loops in an anti-clockwise direction. The trajectory of our Duckiebot based on odometry
and our motion model is shown in the figure below:
<img src="https://github.com/asvath/SLAMDuck/blob/master/pix/wheel_odometry.jpg" width="500" height="400">
The figure highlights the inaccuracy of our odometry (see Section VIII.A for more details.) The Duckiebot does not appear to travel for two loops and it appears to be travelling in a clockwise direction (as opposed to the actual counterclockwise direction) starting at
position (0,0). The goal of running SLAMDuck is to correct this trajectory. 

### Evaluation of SLAMDuck
After running SLAMDuck, we compared our corrected trajectory to the ground truth obtained from vicon. The figure below shows the entirety of Duckiebot’s corrected trajectory in grey. The ground truth is shown in blue. We also plotted our landmark estimates as red circles and the ground truth landmark positions in blue. The ground truth was compared by forcing the t=0 timestep to overlap perfectly with the calculated trajectory. Our corrected trajectory shows the Duckiebot travelling around Duckietown for two loops. While the shape of the trajectory is similar to the ground truth, we see that our corrected trajectory is much bigger. In addition, our landmark positions are far from the ground truth position with average errors in the x and y directions of -0.27 m and -0.23 m respectively.

<img src="https://github.com/asvath/SLAMDuck/blob/master/pix/tracj.jpg" width="500" height="400">


To further analyze our correction, we plotted the errors of the pose of the Duckiebot with respect to the ground truth along
with a three-sigma uncertainty envelope as showcased in the figure below. From the error plots we deduce that our SLAMDuck
produces overconfident estimates of the Duckiebot’s pose in the x and y directions. Our errors do not stay within the uncertainty
envelope and are skewed in one direction. This is probably due to a systematic (i.e. non-normal) error in our range
measurements.

<img src="https://github.com/asvath/SLAMDuck/blob/master/pix/errors.PNG" width="500" height="800">

### Analysis of range measurements
We suspect that our range measurements suffer from a systematic error. The range measurements of AprilTag landmarks closer to the Duckiebot are probably more accurate than those further way due to inverse linear relationship between size and distance. To investigate this, we have run the SLAMDuck algorithm again, but only considering range measurements that are less than 90 cm away from our
Duckiebot as compared to the previous section, where we used all measurements. Figure below shows an improved corrected trajectory of our Duckiebot. Our trajectory shows a smaller, less lopsided loop compared to that in the previous section.

<img src="https://github.com/asvath/SLAMDuck/blob/master/pix/trac_90.png" width="500" height="400">

From the error plots we see an improvement in our estimates of the Duckiebot’s pose in x and y. Both errors are smaller than
in the previous section. In addition, both errors are closer to staying within the three-sigma uncertainty envelope and more
symmetric. However, we see a decreased performance in our heading estimates. We conclude that setting a threshold on the range
measurements improves the accuracy of our result. Correspondingly, we conclude that the nearer the landmark AprilTags are, the more accurate the range measurements are. As part of future work as discussed in Section IX, we should further investigate the threshold for range measurements. We could also improve our accuracy by increasing the number of AprilTag Landmarks and placing them closer together. An improved algorithm which uses a proportional uncertainty for the range measurements may also be possible.

<img src="https://github.com/asvath/SLAMDuck/blob/master/pix/errors_90.PNG" width="500" height="800">

## Videos of Results

This video shows our SLAMDuck trajectory vs. vicon ground truth. We used all of our measurements as discussed in the Evaluation of SLAMDuck section. 

[![Alt text](https://github.com/asvath/SLAMDuck/blob/master/pix/Capture_final_results.PNG)](https://www.youtube.com/watch?v=zgk5YCREUH8)

This video shows our SLAMDuck trajectory vs, vicon ground truth. However, here we only consider range measurements that are less than 90 cm away from our Duckiebot as discussed in Analysis of range measurements. 
[![Alt text](https://github.com/asvath/SLAMDuck/blob/master/pix/Capture_final_results_90.PNG)](https://www.youtube.com/watch?v=OrM_QN1P5fI&t=7s)

## Future work
Possible improvements to our experiment could entail installing wheel encoders to the system. Instead of wheel encoders, we could also try visual odometry to estimate the pose of the Duckiebot during the SLAMDuck prediction step. This approach will not add additional cost to the Duckiebot. We could also add more landmark AprilTags to improve the accuracy ofour estimates. We should also investigate the accuracy of the AprilTags detection library by physically measuring the range from our Duckiebot to the AprilTags and comparing it to the range obtained from the library. Next, the feasibility of running SLAMDuck in real time should be verified.

## Code
This section describes the code used to perform the Data Acquision and preprocessing step:

### a) Write out wheel velocities stored in .bag as txt : my_node.py
Ensure that topics points to the wheel_cmd, e.g : "/luthor/wheels_driver_node/wheels_cmd"
Uncomment : #f.write(str(msg.vel_left) +" " + str(msg.vel_right) + " " + str(msg.header.stamp) + " \n")
The code writes out a text file containing the left and right wheel velocities and the timestamp

### b) Write out images stored in .bag as jpg : my_node.py
Same code as writing out the wheel velocities. However, edit the code to ensure that the topics points to image/compressed
e.g "/luthor/camera_node/image/compressed"
Code outputs colour images from the duckiebot


### c) Write out timestamp of images stored in .bag : my_node.py
Edit the same code used above, this time write out a txt file containing the timestamps
f.write(str(msg.header.stamp) +" \n")

Note that the commends and code are self-explanatory and can be easily edited to perform the above tasks.

### d) Undistort images : undistort_duckie_images.py
The code undistorts the images acquired during data collection and output the undistorted colour images. The duckiebot uses a fisheye lens. However, we use a plumb bob model to perform the undistortion. Ensure that you are using the intrinsics acquired during camera calibration. 

### e) Linear interploation of wheels : interplote_velocity.py
The output wheel velocity and the output images are acquired at 30Hz. As the wheel velocities and the images
were acquired at different timestamps, we linearly interpolated the wheel velocities at the image observations timestamps.

When running the code, change wheel_file_path, image_file_path and out_path to suit your local directories
wheel_file_path: text file of the left and right velocities and the timestamp. This textfile was produced in a)
image_file_path : text file containing the timestamp of the images as described in b)

This code outs 

### f) April Tag detection: apriltags3.py (modified)
We used the publicly available AprilTags detection library apriltags3 https://github.com/duckietown/apriltags3-py to detect the landmark AprilTags in our images and made modifications to store the outputs as jsonfiles. When running this code, ensure that undistorted_images_data_dir points to your undistorted images; and tag_detections_dir is your output dir  (see example json file: tags_detections_undistorted_frame000000.json)

### f) Range and bearing calculation : read_tags.py
Using the relative position of the tags, pose_t,  (stored in the jsonfiles described in f) we then calculated the range and bearing from the camera to the tags. Note that pose_t uses the usual computer vision convention with z-front, x-right, y-down. Our frames are defined according to the covention of z-up, x-right, y-front. All our calculations are expressed relative to our defined frames. The 15-degree tilt of our camera was taken into account during the calculations (rotation matrix called rotation) . For each image, we have the corresponding range and bearing measurements to the AprilTags present in the image. These measurements are stored as txt files per image, containing the tag_id, range and bearing measurements (see example txt file: range_bearing_frame000000.txt)

### g) SLAMDuck.py
Our SLAMDuck algorithm then follows the EKF SLAM algorithm described in S. Thrun, W. Burgard, and D. Fox Probabalistic Robotics (2005). chapter 10 page 314. Where our motion model is used in the prediction step and our measurement model is used in the correction step.
motion model: uses the interpolated left and right wheel velocities, which we stored as text file
measurement model: uses the range and bearing measurements calculated in f)

Our algorithm outputs the estimate of the true pose of the Duckiebot at timesteps k1:k2 and the estimate of the true positions of the N static landmarks with respect to Fw (defined earlier). It also outputs the covariance elipse of the robot and the landmarks.

Note : 
- Play around with these parameters
v_var = 0.5#velocity variance
om_var = 0.5#rotational velocity variance
r_var = 1e-3 #(sensor) range variance
b_var = 1e-3#(sensor) bearing variance


- d = 0.06 #distance between camera and axle 
This value ensures that our range and bearing measurements are actually wrt to the center between the two wheels.




## Duckietown Autonomous Group

<img src="https://github.com/asvath/SLAMDuck/blob/master/pix/squad.png" width="700" height="500">
Duckietown Autonomous Group: (from left) Quackaline Rao, Samantha Quacksome, Dr. Dorothy Duckworthy, Duckiebot Luthor, Lena Luthor and Supergirl.
