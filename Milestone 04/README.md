## Python Files
Before running any of the files, install the dependencies first and make sure ROS2 is sourced in the terminal:
```pip install -r requirements.txt```

```joint_space_trajectory.py``` moves the robot from (0.7, 0.4, 0.3) to (0.3, -0.7, 0.6) using joint space trajectory planning

```task_space_trajectory.py``` moves the robot from (0.7, 0.4, 0.3) to (0.3, -0.7, 0.6) in a straight line using task space trajectory planning

```joint_space_trajectory_calculations.py``` shows the how the joint space trajectory equations were derived using python, and computes the joint values for integer timesteps 0 to 10. No equivalent file exists for task space since it follows a simple straight-line trajectory.

## Videos

[Demo Link](https://drive.google.com/file/d/1dZoLAB5u5eaRTAIn8ae6hHBzzMouQQUN/view?usp=sharing)
