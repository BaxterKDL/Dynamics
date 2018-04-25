from __future__ import print_function
from dynamics import Dynamics
import os
import time

# os.getcwd()

jv = [0.1, 0.2, 0.3, 0.12, 0.42, 0.16, 0.58]

obj = Dynamics(5, jv)

# print obj.pos_COM

I_mat = obj.calc_inertia_mat()

# J_mat = obj.calc_J_i_matrices()
#
# # print ("\n\nJ matrix\n\n", J_mat)
#
joint_pose = {'s0':0, 's1':0, 'e0':0, 'e1':0, 'w0':0, 'w1':0, 'w2':0}
TT = obj.calc_trans_matrices(joint_pose)
# print (TT)

J_matrix = obj.calc_J_i_matrices()

# print (J_matrix)

Uij = obj.calc_U_ij_mat(TT)
# print (Uij)

mm = obj.calc_M_matrix()
# print (mm)

# cc = obj.calc_C_matrix()
# print (cc)

gg = obj.calc_G_matrix()
print (gg)



# aa = time.time()
# for a in xrange(0,7):
#     for b in xrange(0,7):
#         for c in xrange(0,7):
#             print ("a")
#
#
# bb = time.time()
#
# print ("bb-aa", bb-aa)