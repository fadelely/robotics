import sympy as sp

q1, q2, q3, q4, q5, q6 = sp.symbols('q1 q2 q3 q4 q5 q6')

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

P = [x_expr, y_expr, z_expr]

Q = [q1, q2, q3, q4, q5, q6]

J_matrix = sp.Matrix(P).jacobian(Q)

print("--- Jacobian Matrix Elements ---")
for i, p_i in enumerate(['x', 'y', 'z']):
    for j, q_j in enumerate(['q1', 'q2', 'q3', 'q4', 'q5', 'q6']):
        element = J_matrix[i, j]
        print(f"J_({i+1}, {j+1}) : d({p_i})/d({q_j}) = ")
        print(f"  {element}\n")

