from __future__ import print_function
# from dynamics import Dynamics
import os
import time

# os.getcwd()
#
# obj = Dynamics(5)
#
# # print obj.pos_COM
#
# I_mat = obj.calc_inertia_mat()
#
# # print (I_mat[0])
#
# J_mat = obj.calc_J_i_matrices()
#
# # print ("\n\nJ matrix\n\n", J_mat)
#
# joint_pose = {'s0':-130, 's1':-112, 'e0':148.5, 'e1':150, 'w0':175.25, 'w1':120, 'w2':175.25}
#
# obj.calc_trans_matrices(joint_pose)

aa = time.time()
for a in xrange(0,7):
    for b in xrange(0,7):
        for c in xrange(0,7):
            print ("a")


bb = time.time()

print ("bb-aa", bb-aa)