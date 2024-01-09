# Control-of-Robotic-Manipulator
This project was part of the ME404 Robotics Course at the Egyptian Russian University Faculty of Engineering.
The project aims to apply the principles of Forward and Inverse Kinematics to a Robot Manipulator. CoppeliaSim was used as the simulation tool where the UR5 robot manipulator was used to apply kinematics to. The robot is capable of preforming both Forward and Inverse Kinematics, as well as simple Linear Interpolation. The script can be easily edited to allow for more complex tasks such as pick and place.


# Installing zmqRemoteAPI
To be able to run external python files with CoppeliaSim, please follow the instructions listed below:
https://manual.coppeliarobotics.com/en/zmqRemoteApiOverview.htm

# UR5 Schematic
![UR5_Schematic](https://github.com/mazenelgabalawy/Control-of-Robotic-Manipulator/assets/72276135/c90dab51-c18b-4bc3-95a5-911fb1df31d3)

# DH-Table
The lengths are all in meters and all angles are in Radian.
![image](https://github.com/mazenelgabalawy/Control-of-Robotic-Manipulator/assets/72276135/1b68ba23-f4d5-4c2f-9068-2c02be7dcf27)

# Forward Kinematics
Forward kinematics are first calculated analytically using the Denavit-Hartenberg Parameters. The pose of the End-Effector that is obtained through the analytic method, is then compared to that coming from the CoppeliaSim Scene to ensure the DH-table is correct.



https://github.com/mazenelgabalawy/Control-of-Robotic-Manipulator/assets/72276135/cd8891ed-5bfc-4496-9d2a-e03d8c4c78de


# Inverse Kinematics
Inverse kinematics are calculated numerically using the Newton-Raphson Method. The recursive algorithm stops when the error is less than 0.001 meters.

## Real time
Manipulator End-Effector chases after a target dummy in real time.



https://github.com/mazenelgabalawy/Control-of-Robotic-Manipulator/assets/72276135/08de1f99-59b7-4f69-953b-abbbc768c8a5


## Desired Pose
A point dummy is used as the desired pose where the manipulator end-effector should end up.



https://github.com/mazenelgabalawy/Control-of-Robotic-Manipulator/assets/72276135/55ef843e-f676-4972-8aea-73d79642a4c4


# Linear Interpolation
Here we constantly move the End-Effector between three different poses.


https://github.com/mazenelgabalawy/Control-of-Robotic-Manipulator/assets/72276135/d8ffe6fd-797a-42c7-a092-6176f99254c9

