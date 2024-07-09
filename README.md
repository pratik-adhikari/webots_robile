# Robile Description

This repository contains the description files for AST Multirobot implementation in Webots. 

For having Multirobot capability with previous architecture, additionally, two GPS (front and rear side) were added for position and orientation detection. As IMU (Accelerometer & Gyro sensor) data was raw, a required additional controller had to be implemented. Instead, GPS was working ok. 

It was tested with a preplanned distance of 5 meters to the leader robot, and the follower robots had an offset w.r.t leader and had to keep up to the same offset to the leader.

An individual controller for each follower robot is needed, having individual communication port and offset w.r.t leader. Also, the port has to be added to `leader_robot.py` to send data to an additional port.

### Structure
In controller folder, the python based control codes are used for demo, and are named as per working. For this assaignmnet leader_robot and follower_robot are used and world file will load the model and controllers mentioned below.

(Further the Control Drive used to actuate Robile has to be corrected to be omnidirectional and differential drive, which is causing error and limmiting functionality. But for purpose of assaignment it is working.)

### A Demonstration of working has been added.
[Demonstration video](https://github.com/pratik-adhikari/webots_robile/blob/master/Multirobot.mkv)

## Running the Webots Simulation

To run the multi-robot simulation in Webots, follow these steps:

Navigate to the worlds directory:
'''   

cd worlds

webots multi_robot2.wbt
'''
