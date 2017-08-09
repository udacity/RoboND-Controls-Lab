
# Hover Controller #
**Step 1: Writing the PID Class**

Add the PID class you wrote to `quad_controller/src/quad_rotor/pid_controller.py`

**Step 2: Running the Hover Controller**

After you have added the PID class, you can launch the hover controller.
To do so, run the following commands in your terminal:
```
$ cd ~/catkin/ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch quad_controller hover_controller.launch
```
Now, with the hover controller launched, you can launch the quad simulator on your host platform.
The details surrounding this process will be be different for each host platform (Win/Mac/Linux),
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

** Additional Helpful Tools for Debugging/Tuning **
COMING SOON!
