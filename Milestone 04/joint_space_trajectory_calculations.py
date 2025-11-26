import numpy as np
from sympy import symbols, Eq, solve

# Initial joints q(0)
Initial_Q =[-2.78854756, -0.58079889, 1.10202659, -2.09202403, 1.57079633, 0.0]


# Final joints q(T) "which is q(10) in our case"
Final_Q = [-4.48343493, -0.91867686, 0.91839568, -1.57051514, 
        1.57079633, 0.0]

# Total time (we choose 10 but could be altered)
# it is the time the arm takes to reach the final position 
t = 10

# q(t) = c0 + c1*t + c2*t^2 + c3*t^3 , we used third degree polynomial
C0 = Initial_Q # Starting position
C1 = [0, 0, 0, 0, 0, 0, 0]  # initial velocity is 0s 
C2 = [] # to be calculated
C3 = [] # to be calculated
    

# here, we loop over each joint to calculate C2 and C3 coefficients 
for joint_index in range(6):
    # we created symbols for c2 and c3
    c2_tmp, c3_tmp = symbols('c2_tmp c3_tmp')
    # q equation --> q(t) = c0 + c1*t + c2*t**2 + c3*t**3
    eq_1 = Eq(C0[joint_index] + c2_tmp*(t)**2 + c3_tmp*(t)**3, Final_Q[joint_index]) 
    # q_dot equation --> q_dot(t) = c1 + 2*c2*t + 3*c3*t**2 
    eq_2 = Eq(2*c2_tmp*t + 3*c3_tmp*t**2, 0)
    # solve the 2 equations to get c2 and c3
    solved_eq = solve((eq_1, eq_2), (c2_tmp, c3_tmp))
    # append to C2 and C3 lists
    C2.append(solved_eq[c2_tmp])
    C3.append(solved_eq[c3_tmp])

# now we have all the coefficients, c0 ,c1,c2,c3 for all joints
# we can calculate the trajectory at each time step
# Time steps from 0 to 10 with step size 1
time_steps = np.arange(0, 11, 1)

# Initialize list 0f equations 
q_traj = []              # q(t) = c0 + c1*t + c2*t^2 + c3*t^3 --> joint positions
q_dot_traj = []          # q_dot(t) = c1 + 2*c2*t + 3*c3*t^2 --> joint velocities
# q_double_dot_traj = []   # q_double_dot(t) = 2*c2 + 6*c3*t --> joint accelerations


# Calculate equations for all 7 joints at each time step
for joint_index in range(6):
    q_values = [] # Position values
    q_dot_values = [] # Velocity values
    # q_double_dot_values = [] # Acceleration values
    
    # calculate trajectory values at each time step
    for t_curr in time_steps:
        # Position equation
        q_t = C0[joint_index] + C1[joint_index]*t_curr + C2[joint_index]*t_curr**2 + C3[joint_index]*t_curr**3

        # Velocity equation 
        q_dot_t = C1[joint_index] + 2*C2[joint_index]*t_curr + 3*C3[joint_index]*t_curr**2
        
        # Acceleration equation 
        # q_double_dot_t = 2*C2[joint_index] + 6*C3[joint_index]*t_curr
        
        # we add the q values for each joint (q0 --> q6)
        # so it will look like [q0(0), q0(1), q0(2) ..etc] then [q1(0), q1(1), q1(2) ..etc] and so on
        q_values.append(q_t)
        # same here, 
        # [q0_dot(0), q0_dot(1), ..etc] then [q1_dot(0), q1_dot(1) ..etc] and so on
        q_dot_values.append(q_dot_t)
        # same here, 
        # [q0_double_dot(0), q0_double_dot(1), ..etc] then [q1_double_dot(0), q2_double_dot(1) ..etc]  and so on
        # q_double_dot_values.append(q_double_dot_t)
    
    # then, we add the list of each joint to the main list so it will look like [[q0 .. q6 at t=0], [q0 .. q6 at t=1] ..etc]
    q_traj.append(q_values)
    # same here , [[q1_dot .. q6_dot at t=0], [q1_dot .. q6_dot at t=1] ..etc]
    q_dot_traj.append(q_dot_values)
    # same here , [[q1_double_dot .. q6_double_dot at t=0], [q1_double_dot .. q6_double_dot at t=1] ..etc]
    # q_double_dot_traj.append(q_double_dot_values)

# Print results for each joint
print("  q_c2:", [round(x, 6) for x in C2])
print("  q_c3:", [round(x, 6) for x in C3])

for joint_index in range(6):
    print(f"\nJoint {joint_index}:")
    print(f"  q(t):            {[round(val, 4) for val in q_traj[joint_index]]}")
    print(f"  q_dot(t):        {[round(val, 4) for val in q_dot_traj[joint_index]]}")
    # print(f"  q_double_dot(t): {[round(val, 2) for val in q_double_dot_traj[joint_index]]}")
