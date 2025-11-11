import sympy as sp

q1, q2, q3, q4, q5, q6 = sp.symbols('q1 q2 q3 q4 q5 q6', real=True)
qd1, qd2, qd3, qd4, qd5, qd6 = sp.symbols('qd1 qd2 qd3 qd4 qd5 qd6', real=True)

x_expr = (
    0.1 * sp.sin(q1) * sp.cos(q5)
    + 0.127 * sp.sin(q1)
    - 0.1 * sp.sin(q5) * sp.cos(q1) * sp.cos(q2 + q3 + q4)
    + 0.1 * sp.sin(q2 + q3 + q4) * sp.cos(q1)
    - 0.425 * sp.cos(q1) * sp.cos(q2)
    - 0.392 * sp.cos(q1) * sp.cos(q2 + q3)
)

y_expr = (
    -0.1 * sp.sin(q1) * sp.sin(q5) * sp.cos(q2 + q3 + q4)
    + 0.1 * sp.sin(q1) * sp.sin(q2 + q3 + q4)
    - 0.425 * sp.sin(q1) * sp.cos(q2)
    - 0.392 * sp.sin(q1) * sp.cos(q2 + q3)
    - 0.1 * sp.cos(q1) * sp.cos(q5)
    - 0.127 * sp.cos(q1)
)

z_expr = (
    -0.425 * sp.sin(q2)
    - 0.1 * sp.sin(q5) * sp.sin(q2 + q3 + q4)
    - 0.392 * sp.sin(q2 + q3)
    - 0.1 * sp.cos(q2 + q3 + q4)
    + 0.163
)

P = sp.Matrix([x_expr, y_expr, z_expr])
Q = [q1, q2, q3, q4, q5, q6]
Qd = [qd1, qd2, qd3, qd4, qd5, qd6]

J = P.jacobian(Q)

Jdot = sp.zeros(*J.shape)
for qi, qdi in zip(Q, Qd):
    Jdot += J.diff(qi) * qdi

Jdot = sp.simplify(Jdot)

print("\n--- Time Derivative of Jacobian (Jdot) ---")
for i in range(Jdot.rows):
    for j in range(Jdot.cols):
        print(f"Jdot_({i+1},{j+1}) = ")
        print(f"{Jdot[i,j]}\n")
