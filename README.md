# Improved camera calibration with a planar pattern using OpenCV
Implementation project of camera calibration with planar patters using OpenCV library. This project was based on the following papers.

1. [A Flexible New Technique for Camera Calibration] (https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/tr98-71.pdf)
2. [Accurate Camera Calibration using Iterative Reﬁnement of Control Points] (https://www.ri.cmu.edu/pub_files/2009/10/Calib.pdf)
3. [Enhanced Camera Calibration for Machine Vision using OpenCV] (https://www.researchgate.net/publication/305636550_Enhanced_Camera_Calibration_for_Machine_Vision_using_OpenCV)

The report of this implementation is in Camera_Calibration.pdf at the root of the repository.

## Description
There are three projects in this repository.


1. Pattern Detection: This take an input video or real time capturing and identify and track the control points in the pattern. Chessboard, Grid Circles and Rings are soported.


2. Frame Selection: This part implements an algorithm that select a set of images from an input video o real time capturing, and selects the most appropiate images to calibrate a camera.


3. Camera Calibration: In this part is implemented the calibration algorithm itself following the papers mentioned above.     

## Requirements
This project only needs OpenCV 4.0 library 

## Build steps
The project was tested on Ubuntu 18.0. 
Enter to any of the individual projects and run the following commands.

	mkdir build && cd build
	cmake .. && make 
	./output
  
