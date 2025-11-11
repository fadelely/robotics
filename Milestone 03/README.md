## Python Files
Before running any of the files, install the dependencies first:
```pip install -r requirements.txt```

```forward_velocity_publisher.py``` takes the joint velocities as input -> outputs end-effector velocities and sends the joint velocities to the robot arm.

```inverse_velocity_publisher.py``` takes the end-effector velocities as input -> outputs joint velocities and sends them to the robot arm.

```forward_inverse_velocity.py``` takes the joint positions and joint velocities to compute the forward velocity kinematics. It then takes the end-effector velocities and uses it along side the previously given joint positions to compute the inverse velocity kinematics. All inputs are given by the user.

```forward_inverse_acceleration.py``` takes the joint positions, joint velocities, and joint accelerations to compute the forward acceleration kinematics. It then takes the end-effector accelerations and uses it along side the previously given joint positions and joint velocities to compute the inverse acceleration kinematics. All inputs are given by the user.

```jacobian_matrix.py``` computes the symbolic form of the jacobian matrix and prints each element in the terminal.

```jacobian_dot_matrix.py``` computes the differentiated jacobian matrix and prints each element in the terminal.
## Videos
