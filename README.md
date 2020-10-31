# Demo 1 README
The objective of this demo is to perform a few tests.  These include a CV test to measure the angle of an Aruco marker on a captured photograph, a test that involves the robot travelling a linear distance, and one that involves the robot rotating a certain number of degrees.

### CV Code
The CV code is contained in `aruco_detect.py`. This code is an object-oriented set of functionality. At the end of the file, an instance of the object is created in order to run the CV test.

###  Robot Test
The code for these tests are contained in a foldercalled `ArduinCode` in `demo1.ino` and in `demo1_rotation.ino`. The first of these files is used to move the robot a certain distance. The second allows the robot to be rotated a certain distance. 

### Miscellaneous Files
The repository also contains a file named `marker1.jpg` which is just an Aruco marker that is easily accessible to anyone with repository access for testing purposes. There are also some simulink diagrams in the repo from the design of the PID controller. These are contained in `Simulink Diagrams`.
