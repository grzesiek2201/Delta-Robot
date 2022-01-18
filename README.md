# DIY Delta robot

## For your daily needs

**This repo gives a brief (for now) explanation about designing, modeling, building, wiring and programming delta type robot.**

### What does this repository include?
- Application controlling the robot with **live visualization** of the kinematics written in Python
- 3D model of the whole robot, as well as 3D models of all the parts that need to be printed
- BOM
- Build instructions

### Pictures and screenshots presenting the robot and application

| Real-life picture of the robot | Main screen of the application |
|            :---:               |            :---:               |
| <img src="https://user-images.githubusercontent.com/84570140/149950226-a1bb82dc-97a9-4bc8-ab05-011d34f5940a.jpg" width=30% height=30%> | <img src="https://user-images.githubusercontent.com/84570140/149950918-80e215e9-9a54-49c9-802b-f81aedbbf43c.png" width=100% height=100%> |

### Features
- Both online and offline working mode - in offline you can simulate the robot's configuration in different positions
- Basic manual control by providing x,y,z coordiantes or $θ_1$, $θ_2$, $θ_3$
- Basic relative position control in Jog mode
- Enabling/disabling motors
- Starting/stoping the program
- Encoders calibrating
- Homing
- Changing TCP offset and Jog step
- Changing Z axis limit
- Live robot kinematics visualization
- Live TCP displacement plot in x,y,z coordinates
- Live joints displacement plot in angular coordinates
- Creating and manipulating robot programs, saving, opening and editing. Points can be set either by manually typing in the coordinates or by a teach-in method, in which you hold the robot in a certain position and take it's current configuration as a point's coordiante
