# ForwardKinematic
An early experiment in using kinematics to calculate a differential drive robot's position and heading given the speeds of the wheels. As this project is of an experimental nature, most of the code is rather messy and disorganized, with a few exceptions as follows:

### ForwardKinematic.java
This class houses the core of the programs functionality and is relatively well documented and cleanly written, unlike the rest of this project. This class uses some basic Kinematic equations taken from Magnus Egerstedt's online course, Control of Mobile Robots, taught at the School of Electrical and Computer Engineering at the Georgia Institute of Technology. Thanks!

Using two control parameters, the left and right wheel velocities, this class is able to use the aforementioned kinematic equations to calculate the instantaneous change in position and heading of the robot. These values are then integrated using a small timestep (to simulate the 20ms refresh rate of the periodic loops on the roboRio) to closely approximate the cartesian position and heading of the robot.

This class is designed to be easily dropped into actual competition software with few modifications.

### CSVHelper.java
A byproduct of this project was this simple and sloppily written class to provide an easy wasy to deal with values stored in .CSV files. The two main operations are read and write to a CSV file, providing a simple way to turn 2dimensional double arrays into .CSV files and vice versa. This still needs a lot of work but it is a start in dabbling with CSV's, which will undoubtedly be very useful in the coming season.
