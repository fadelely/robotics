import sympy as sp
import numpy as np

q1, q2, q3, q4, q5, q6 = sp.symbols('q1 q2 q3 q4 q5 q6')

x = (
    0.1 * sp.sin(q1) * sp.cos(q5)
    + 0.127 * sp.sin(q1)
    - 0.1 * sp.sin(q5) * sp.cos(q1) * sp.cos(q2 + q3 + q4)
    + 0.1 * sp.sin(q2 + q3 + q4) * sp.cos(q1)
    - 0.425 * sp.cos(q1) * sp.cos(q2)
    - 0.392 * sp.cos(q1) * sp.cos(q2 + q3)
)

y = (
    -0.1 * sp.sin(q1) * sp.sin(q5) * sp.cos(q2 + q3 + q4)
    + 0.1 * sp.sin(q1) * sp.sin(q2 + q3 + q4)
    - 0.425 * sp.sin(q1) * sp.cos(q2)
    - 0.392 * sp.sin(q1) * sp.cos(q2 + q3)
    - 0.1 * sp.cos(q1) * sp.cos(q5)
    - 0.127 * sp.cos(q1)
)

z = (
    -0.425 * sp.sin(q2)
    - 0.1 * sp.sin(q5) * sp.sin(q2 + q3 + q4)
    - 0.392 * sp.sin(q2 + q3)
    - 0.1 * sp.cos(q2 + q3 + q4)
    + 0.163
)

p = [x, y, z]

q = [q1, q2, q3, q4, q5, q6]

# Calculate the 3x6 Position Jacobian matrix
J = sp.Matrix(p).jacobian(q)

# User input for joint angles only
q_vals = [float(x) for x in input("Enter joint angles q1-q6 (radians, space separated): ").split()]

# Create substitution dictionary for symbolic evaluation
subs = {q[i]: q_vals[i] for i in range(6)}

J_num = np.array(J.evalf(subs=subs), dtype=np.float64)

# Forward velocity kinematics
dq_vals = np.array([float(x) for x in input("Enter joint velocities dq1-dq6 (rad/s, space separated): ").split()]).reshape(-1,1)
v_ee = J_num @ dq_vals
print("End-effector linear velocity [vx, vy, vz] (m/s):", v_ee.flatten())

# Inverse velocity kinematics
v_input = np.array([float(x) for x in input("Enter desired end-effector velocities vx vy vz (m/s, space separated): ").split()]).reshape(-1,1)
J_pinv = np.linalg.pinv(J_num)
dq_solution = J_pinv @ v_input
print("Required joint velocities [dq1-dq6] (rad/s):", dq_solution.flatten())
