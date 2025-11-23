import numpy as np
import sympy as sp

# Initial joints 
Initial_Q = [-2.788547563050074, -0.5807988920173397, 1.102026594222845, 
            -2.0920240290004024, 1.5707963267948966, -1.9238414173346157, 0.0]

# Final joints
Final_Q = [-4.48343493, -0.91867686, 0.91839568, -1.57051514, 
        1.57079633, -0.22895405, 0.0]

# Total time (0we choose 10 but could be altered)
t = 10

# q(t) = c0 + c1*t + c2*t^2 + c3*t^3 , we used third degree polynomial
C0 = Initial_Q # Starting position
C1 = [0, 0, 0, 0, 0, 0, 0]  # initial velocity is 0s
# C2 = [0, 0, 0, 0, 0, 0, 0]  # initial acceleration is 0s
# C3 = [(Final_Q[i] - C0[i]) / (t**3) for i in range(7)]  # calculate c3 for each joint

C2 = []
C3 = []
    
from sympy import symbols, Eq, solve

for joint_index in range(7):
    c2_tmp, c3_tmp = symbols('c2_tmp c3_tmp')
    # q equation
    eq_1 = Eq(C0[joint_index] + c2_tmp*(t)**2 + c3_tmp*(t)**3, Final_Q[joint_index]) 
    # q_dot equation
    eq_2 = Eq(2*c2_tmp*t + 3*c3_tmp*t**2, 0)
    #solve
    solved_eq = solve((eq_1, eq_2), (c2_tmp, c3_tmp))
    # append to C2 and C3 lists
    C2.append(solved_eq[c2_tmp])
    C3.append(solved_eq[c3_tmp])


# Time steps from 0 to 10 with step size 1
time_steps = np.arange(0, 11, 1)

# Initialize list 0f equations 
q_traj = []              # q(t) = c0 + c1*t + c2*t^2 + c3*t^3
q_dot_traj = []          # q_dot(t) = c1 + 2*c2*t + 3*c3*t^2
# q_double_dot_traj = []   # q_double_dot(t) = 2*c2 + 6*c3*t


# Calculate equations for all 7 joints
for joint_index in range(7):
    q_values = [] # Position values
    q_dot_values = [] # Velocity values
    # q_double_dot_values = [] # Acceleration values
    
    for t_curr in time_steps:
        # Position equation
        q_t = C0[joint_index] + C1[joint_index]*t_curr + C2[joint_index]*t_curr**2 + C3[joint_index]*t_curr**3

        # Velocity equation 
        q_dot_t = C1[joint_index] + 2*C2[joint_index]*t_curr + 3*C3[joint_index]*t_curr**2
        
        # Acceleration equation 
        q_double_dot_t = 2*C2[joint_index] + 6*C3[joint_index]*t_curr
        
        # we add the of the q values (q0 --> q6) at t curr so it will look like [q0,q1,...q6] at t=0, then at t=1 ..etc
        q_values.append(q_t)
        # same here, [q1_dot,q2_dot,...q6_dot] at t=0, then at t=1 ..etc
        q_dot_values.append(q_dot_t)
        # same here, [q1_double_dot,q2_double_dot,...q6_double_dot] at t=0, then at t=1 ..etc
        # q_double_dot_values.append(q_double_dot_t)
    
    # then, we add the list of each joint to the main list so it will look like [[q0 .. q6 at t=0], [q0 .. q6 at t=1] ..etc]
    q_traj.append(q_values)
    # same here , [[q1_dot .. q6_dot at t=0], [q1_dot .. q6_dot at t=1] ..etc]
    q_dot_traj.append(q_dot_values)
    # same here , [[q1_double_dot .. q6_double_dot at t=0], [q1_double_dot .. q6_double_dot at t=1] ..etc]
    # q_double_dot_traj.append(q_double_dot_values)

# Print results for each joint
print("  q_c2:", [round(x, 2) for x in C2])
print("  q_c3:", [round(x, 2) for x in C3])

for joint_index in range(7):
    print(f"\nJoint {joint_index}:")
    print(f"  q(t):            {[round(val, 2) for val in q_traj[joint_index]]}")
    print(f"  q_dot(t):        {[round(val, 2) for val in q_dot_traj[joint_index]]}")
    # print(f"  q_double_dot(t): {[round(val, 2) for val in q_double_dot_traj[joint_index]]}")