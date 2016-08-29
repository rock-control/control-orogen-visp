# Visual Servoing Controller based on visp library.

## General Presentation

This repository is an Orocos component for the visual servoing purposes. 
This component is based on the [ViSP library](https://visp.inria.fr).

## Visual Servoing 

The visual servoing mission consists in control the vehicle using data provided by a camera.

### Default coordinate frames

The default coordinate frame is shown on the figure below:

<div style="text-align:center"><img src ="doc/visp_setpoint_diagram.png"/></div>

**Remark**: The object frame depends on the order of the input corners, so the visp assume that
the object has zero degrees of rotation around its z-axis if the corners are provided in 
*clockwise stating by the lower left point*.

### Input Ports
The component has the following inputs:

**cmd_in** - The cmd_in is the desired position (setpoint) of the vehicle in the object frame. 
           So for instance, if its is desired the vehicle stay a distance "d" meters in front
           a certain target, the default setpoint is:

           `setpoint = {x: d, y: 0, z:0, roll: -PI/2, pitch: -PI/2, yaw: 0}` 
**marker_corners** - vector of corners coordinates provided by a certain corner detector. (Look remark on the previous section)


### Relevant Properties:

**Architecture** - Select the desired visual servoing architecture: IBVS, PBVS or HVS(2 1/2D).  
**target_list** - List of the targets parameters (size and rotation around z)
                The rotation around z allows rotate the object frame if the corners are not given in the default value.
                (it might happen that the target detector do not provide the corners in the expected order, or the 
                target is rotated phisically and the roll degree of freedom is not important).  
**desired_target** - Target identifier in which the robot will perform the visual Servoing

#### Camera to body transformation
Visp originally performs the visual servoing taking the camera frame in respect to the body frame. In order to control the 
body frame, a transformation body-to-camera is required. This is applied using the transformer feature of the rock framework.

### Output ports
**cmd_out** - Setpoint of velocities of the six degrees of freedom. The velocities are zero if the desired marker is not detected. 
**controller_state** - Status of the controller (desired_pose, current_pose, error, residual and timestamp)


