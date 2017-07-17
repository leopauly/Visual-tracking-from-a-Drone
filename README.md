# Visual-tracking-from-a-Drone
Code for visual tracking of objects on ground from a drone.

The objects on the ground are tracked based on the following features:
1. Colour : Each object is given a purticular color
2. Size: The area of the object
3. Shape : The objects are further filtered out using a roundness parameter 
4. Distance between same obejects in adjacent frames : This criteria helps to discard tracking of objects which moves beyond a purticular distance between the frames

Tested on:
Jetson TX1 running on Ubuntu16.04 with an attached ZED camera being carried on a drone.

Directories:
1. OpenCV : Contains the code of the first version of the tracking algorithm 
2. OpenCV_live : Contains modified code for deploying the algorithm in realtime in a drone
3. OpenCV_video : Contains code for testing the algorithm on a prerecorded video 
4. Results_Demo : Has demo videos of our algorithm working on pre-recorded videos


How to use these programs:
1. Download the repositories
2. Open the repository
3. Delete the build folder
4. Use the following commands to build the code from source file:
 $ mkdir build
 $ cd build
 $ cmake ..
 $ make 
5. Use the following command to run the binary file:
 $ ./ZED_with_OpenCV


NB:  The code need to be cleaned up a lot. I will try to do it when I get some free time.
