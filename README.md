# Controller abstract interface

This package provides an abstract interface to the RENOIR robot with a simple example.
The aim of this abstract interface is to handle various implementations of controllers and 
load them in the RENOIR robot (for now we have the TALOS robot).

The nominalSetSensors method provides the sensor information from the robot.
A map of double vectors allows differenciating betwen IMU data, force data, encoders and so on.

The getControl method provides the call to compute the control law.
The user is expected to fill in the map of control double vector values.


# Cloning, building and installing

```
    mkdir -p tirrex_renoir_ws/src
    cd tirrex_renoir_ws/src
    git clone https://github.com/olivier-stasse/renoir_controller_abstract_interface.git
    catkin build renoir_controller_abstract_interface
```

# Testing 

There is a simple node loading an interface and making a simple call to each abstract method 
(setupSetSensors, nominalSetSensors, cleanupSetSensors, getControl).

```
    rosrun renoir_controller_abstract_interface load_renoir_controller_node ./devel/lib/librenoir_controller_example_interface.so
```    

The current expected output is:
```
Calling /home/user/tirrex_renoir_ws/devel/lib/renoir_controller_abstract_interface/load_renoir_controller_node with ./devel/lib/librenoir_controller_example_interface.so
Tried loading 
Succedded in loading ./devel/lib/librenoir_controller_example_interface.so
 setupSetSensors - Size of sensorsIn:5
 nominalSetSensors - Size of sensorsIn:5
 cleanupSetSensors - Size of sensorsIn:5
 getControl - Size of controlOut:5
```
