# Visual-tracking-from-a-Drone
Code for visual tracking of objects on ground from a drone.


The objects are tracked on the ground based on following 


Directories:

1. OpenCV : Contains the code of the first version of the tracking algorithm 
2. OpenCV_live : Contains modified code for deploying the algorithm in realtime in a drone
3. OpenCV_video : Contains code for testing the algorithm on a prerecorded video 
4. Results_Demo : Has demo videos of our algorithm working on pre-recorded videos

For using this programs:

1.Download the repositories.
2.Open the repository
3.Delete the build folder.
4.Use the following commands to build the code from source file:
$ mkdir build
$ cd build
$ cmake ..
$ make 
5.Use the following command to run the binary file:
$ ./ZED_with_OpenCV
