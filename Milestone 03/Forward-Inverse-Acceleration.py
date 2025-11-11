import sympy as sp
import numpy as np

# Define symbolic variables
q1, q2, q3, q4, q5, q6 = sp.symbols('q1 q2 q3 q4 q5 q6')
q = [q1, q2, q3, q4, q5, q6]

# Position equations
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

# Jacobian (3x6)
J = sp.Matrix(p).jacobian(q)

# Jacobian time derivative (3x6) - CORRECTED
dq = sp.symbols('dq1 dq2 dq3 dq4 dq5 dq6')
J_dot = sp.zeros(3, 6)
for i in range(3):      # For each row
    for j in range(6):  # For each column
        J_dot[i, j] = sum(sp.diff(J[i, j], q[k]) * dq[k] for k in range(6))

# Numeric input
q_vals = [float(x) for x in input("Enter joint angles q1-q6 (radians, space separated): ").split()]
dq_vals = np.array([float(x) for x in input("Enter joint velocities dq1-dq6 (rad/s, space separated): ").split()])

subs = {q[i]: q_vals[i] for i in range(6)}
subs.update({dq[i]: dq_vals[i] for i in range(6)})

J_num = np.array(J.evalf(subs=subs), dtype=np.float64)
J_dot_num = np.array(J_dot.evalf(subs=subs), dtype=np.float64)

# ---- Forward Acceleration ----
ddq_vals = np.array([float(x) for x in input("Enter joint accelerations ddq1-ddq6 (rad/s^2, space separated): ").split()]).reshape(-1,1)
dq_vals = dq_vals.reshape(-1,1)

a_ee = J_dot_num @ dq_vals + J_num @ ddq_vals
print("End-effector linear acceleration [ax, ay, az] (m/s^2):", a_ee.flatten())

# ---- Inverse Acceleration ----
a_input = np.array([float(x) for x in input("Enter desired end-effector accelerations ax ay az (m/s^2, space separated): ").split()]).reshape(-1,1)
J_pinv = np.linalg.pinv(J_num)
ddq_solution = J_pinv @ (a_input - J_dot_num @ dq_vals)
print("Required joint accelerations [ddq1-ddq6] (rad/s^2):", ddq_solution.flatten())
