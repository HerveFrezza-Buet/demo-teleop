# demo-teleop
key_teleop.py is very simple ROS keyboard teleop. We also provide a teleop dedicated for controlling a drone (safe_drone_teleop.py). This teleop catches cmd_vel commands, allowing to interupt an external control with the keybord.

# examples for key_teleop.py

rosrun demo_teleop key_teleop.py

rosrun demo_teleop key_teleop.py --persist

rosrun demo_teleop key_teleop.py /cmd_vel:=/some_input_cmdvel_topic


# examples for safe_drone_teleop.py


rosrun demo_teleop safe_drone_teleop.py reset:=/thedrone/reset takeoff:=/thedrone/takeoff land:=/thedrone/land cmd_vel_out:=/thedrone/cmd_vel

rosrun my_package my_drone_controller cmd_vel:=cmd_vel_in


The keyboard allows for taking off and land. When it flyes, the controller controls the drone, but only if no key has been pressed during the last second. You can read the status of the safe_drone_teleop :

rostopic echo status



