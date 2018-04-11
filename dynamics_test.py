from dynamics import Dynamics
import os

os.getcwd()

obj = Dynamics(5)

print obj.pos_COM

I_mat = obj.calc_inertia_mat()

print (I_mat[0])

J_mat = obj.calc_J_i_matrices()

print (J_mat[0])