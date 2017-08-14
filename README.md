[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

# Hover Controller #
**Step 1: Writing the PID Class**

Add the PID class you wrote to `quad_controller/src/quad_controller/pid_controller.py`

**Step 2: Running the Hover Controller**

**NOTE: Launch `roscore` first, then Unity**

After you have added the PID class, you can launch the hover controller.
To do so, run the following commands in your terminal:
```
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ roslaunch quad_controller hover_controller.launch
```
Now, with the hover controller launched, you can launch the quad simulator on your host platform.
The details surrounding this process will be be different for each host platform (Win/Mac/Linux).
please see "Using the Simulator" below.

**Step 3: Tuning Parameters**
Now that Unity has been launched, verify that you can see poses being published by the simulator
on the `/quad_rotor/pose` topic:
```
$ rostopic echo /quad_rotor/pose
```
If you see messages being published, you're golden! If you don't see messages being published,
then you might want to check your firewall settings on your host VM, or try restarting
the simulator (at the moment there's an infrequent bug where comunications with the simulator does not work).

Now that you've verified poses are being published, you can use `rqt_reconfigure` to adjust the setpoints
and PID parameters for the hover controller!

You should tune the parameters using rqt_reconfigure until you are happy with the result.
For more information about how to tune PID parameters, head back into the classroom.

If manually adjusting the PID parameters is not your thing, feel free to try out the
Ziegler–Nichols tuning method using either `hover_zn_tuner_node`, or `hover_twiddle_tuner_node`.
These nodes both implement tuning methods that were discussed in the classroom.

You might find that these tuner helpers only get you part way to a good tune.
There's a reason for this! Do you know?

To run `hover_zn_tuner_node`:
```
$ rosrun quad_controller hover_zn_tuner_node
```

To run `twiddle_tuner_node`:
```
$ rosrun quad_controller hover_twiddle_tuner_node
```

Write down the parameters you came up with, as you will need them later, for the full positional controller!

**Bonus Quesion:**

Why does the quad overshoot more frequently when heading to a set point at a lower altitude?
How can you modify the code to overcome this issue?

# Attitude Controller #
The attitude controller is responsible for controlling the roll, pitch, and yaw angle of the quad rotor.
You can tune it very similarly to how you tuned the hover controller!
Note: ZN/Twiddle Tuner nodes only work with the Hover controller.

**Step 1: Launch the Attitude Controller**
```
$ roslaunch quad_controller attitude_controller.launch
```

**Step 2: Launch the Attitude Controller**

Tune roll and pitch PID parmaeters until things look good.
You'll also need to write down these PID parameters.
As mentioned previously, you'll be using them in the positional controller!


# Position Controller #

Finally, now that you have tuned the attitude controller and positional controllers
you can begin to work on the positional controller. The positional controller is
responsible for commanding the attitude and thrust vectors in order to acheive a
goal orientation in three dimensional space.

Luckily, you've likely found some pretty decent PID paramters in your previous exploration.
If you're fortunate, these parameters will work for you... However, even if they don't
we've got you covered. There's all sorts of additional tooling that you can use to
troubleshoot and tune positional control of your quad!

**Step 1: Launch the Positional Controller**
```
$roslaunch quad_controller position_controller.launch
```

**Step 2: Tuning Parameters** 

Tune parameters until the controller is well-behaved.
This should not be a very familiar, albeit potentially more difficult problem.

**Step 3: Test Against a Desired Trajectory**

COMING SOON!

**Step 4: Plotting and Scoring** 

COMING SOON!

# Additional Helpful Tools for Debugging/Tuning #
With so many degrees of freedom, debugging and troubleshooting can be a painful process.
In order to make things a little bit simpler, we've provided some tools that might make
life a little bit easier.

**Constraining Forces and Torques**

It is possible to constrain forces and torques on the quad rotor's body frame.
This can be useful if you're trying to debug only a single degree of freedom.

Example: Dissalow movement along the quad's X axis
```
$ rosservice call /quad_rotor/x_force_constrained "data: true"
```
Example: Dissallow rotation about the quad's X axis
```
$ rosservice call /quad_rotor/x_torque_constrained "data: true"
```

**Setting the Camera Pose**

To set the camera pose you can either, right click in the simulator, and drag
or you can use the following service call, where the data parameter may take on the following
values:
0: Orthogonal to the inertial frame's YZ plane, facing the positive X direction.
1: Orthogonal to the inertial frame's XZ plane, facing the positive Y direction.
2: Orthogonal to the intertial frame's XY plane, facing the negative Z direction.
3: Perspective view, facing the quad's body frame origin from the -X,-Y, +Z quadrant.

```
$ rosservice call /quad_rotor/camera_pose_type "data: 0"
```
To reset the camera pose, to the default pose, you ca use the service call, or right click.

**Setting the Camera Distance**

To set the distance between the camera and the quad's body frame, you can use the
`/quad_rotor/camera_distance` service. For example, to set the camera distance to be
20 meters, you would call the service as follows:
```
$ rosservice call /quad_rotor/camera_distance "data: 20.0"
```

To reset the camera distance to the default, simply right click in the simulator.

**Disabling Gravity**

Gravity can be a harsh reality. Particularly when you're dealing with attitude tuning.
Fortunately, we can disable gravity in the simulator for the purposes of debugging.
To so, call the `/quad_rotor/gravity` service as follows:
```
$ rosservice call /quad_rotor/gravity "data: false"
```

**Setting Pose**

To set the quad pose, use the `/quad_rotor/set_pose` service. The following service call
will place the quad at the origin:
```
$ rosservice call /quad_rotor/set_pose "pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0" 
```

# Using the Simulator #

First be sure to grab the newest version of the simulator for your host computer OS [here](https://github.com/udacity/RoboND-Controls-Lab/releases). **Please note that you cannot use the simulator inside of the Udacity supplied course VM (Virtual Machine). You have to use the simulator for your host operating system and connect it to your VM**

If using the VM, inside the simulator's `_data` or `/Contents` folder, edit `ros_settings.txt` and set `vm-ip` to the VM’s IP address and set Override to `true`. If not using a VM, no edit is necessary.

To find the ip of your VM, type `ifconfig  | grep 'inet addr:'| grep -v '127.0.0.1' | cut -d: -f2 | awk '{ print $1}'` into a terminal of your choice
