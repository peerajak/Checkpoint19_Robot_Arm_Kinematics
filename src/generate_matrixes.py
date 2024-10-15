#!/usr/bin/env python3

from sympy import Matrix, cos, sin, Symbol, simplify, trigsimp
from sympy.interactive import printing


# To make display prety
printing.init_printing(use_latex = True)

# List of all DH params in generic form
theta_i = Symbol("theta_i") #i=2,3
r_i = Symbol("r_i")


#i in {1,2,3} //{} is math symbol of a set
DH_Matric_Generic_0_i = Matrix([[cos(theta_i),0,sin(theta_i), 0],
                            [sin(theta_i),  0,-cos(theta_i), 0],
                            [0, 1, 0, 0],
                            [0, 0, 0, 1]])

DH_Matric_Generic_iminus1_i = Matrix([[cos(theta_i), -sin(theta_i), 0, r_i*cos(theta_i)],
                            [sin(theta_i), cos(theta_i), 0, r_i*sin(theta_i)],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

result_simpl_0 = simplify(DH_Matric_Generic_0_i)
result_simpl_i = simplify(DH_Matric_Generic_iminus1_i)

from sympy import preview

# Save to local file
preview(result_simpl_0, viewer='file', filename="out_0.png", dvioptions=['-D','300'])
preview(result_simpl_i, viewer='file', filename="out_i.png", dvioptions=['-D','300'])

# Now create A01, A12, A23
# break down of List of all DH params 
theta_1 = Symbol("theta_1") #i=1 
theta_2 = Symbol("theta_2") #i=2
theta_3 = Symbol("theta_3") #i=3 

# r_1 = 0
r_2 = Symbol("r_2")
r_3 = Symbol("r_3")


A01 = DH_Matric_Generic_0_i.subs(theta_i, theta_1)
A12 = DH_Matric_Generic_iminus1_i.subs(r_i,r_2).subs(theta_i, theta_2)
A23 = DH_Matric_Generic_iminus1_i.subs(r_i,r_3).subs(theta_i, theta_3)

A03 = A01 * A12 * A23
A02 = A01 * A12

A03_simplify = trigsimp(A03)
A02_simplify = trigsimp(A02)

# We save

preview(A03, viewer='file', filename="A03.png", dvioptions=['-D','300'])
preview(A02, viewer='file', filename="A02.png", dvioptions=['-D','300'])

preview(A01, viewer='file', filename="A01.png", dvioptions=['-D','300'])
preview(A12, viewer='file', filename="A12.png", dvioptions=['-D','300'])
preview(A23, viewer='file', filename="A23.png", dvioptions=['-D','300'])
preview(A03_simplify, viewer='file', filename="A03_simplify.png", dvioptions=['-D','300'])
preview(A02_simplify, viewer='file', filename="A02_simplify.png", dvioptions=['-D','300'])