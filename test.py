import sympy as sp


c1, s1, c4, s4 = sp.symbols('c1 s1 c4 s4')

R_0_4 = sp.Matrix([
    [0, -c1, s1],
    [0 , -s1, -c1],
    [1, 0,0],
])

R_z1 = sp.Matrix([
    [c1, -s1, 0],
    [s1 , c1, 0],
    [0, 0,1],
])

R_z4 = sp.Matrix([
    [c4, -s4, 0],
    [s4 , c4, 0],
    [0, 0,1],
])
R_4_e = sp.Matrix([
    [0, -1, 0],
    [-1 , 0, 0],
    [0, 0,-1],
])

R = sp.Matrix([
    [1, 0, 0],
    [0 , 0, 1],
    [0, 1,0],
])

Rx1 = sp.Matrix([
    [1, 0, 0],
    [0 , c1, -s1],
    [0, s1,c1],
])

Rx4 = sp.Matrix([
    [1, 0, 0],
    [0 , c4, -s4],
    [0, s4,c4],
])

#R_4_e_z = R_z @ R_4_e   
#R_0_e = R_0_4 @ R_4_e_z

Rr = R_z1@R@R_z4

print(Rr)