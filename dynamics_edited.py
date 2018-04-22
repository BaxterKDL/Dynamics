from __future__ import print_function
import numpy as np
from FKSolver import FKSolver as fk
from robot import Baxter
from numpy.linalg import inv

class Dynamics:

    def __init__(self,a):

        '''Declaring variables for the  MCG matrices'''

        self.pos_COM = np.array([[-0.05117, 0.07908, 0.00086],
                                 [0.00269, -0.00529, 0.06845],
                                 [-0.07176, 0.08149, 0.00132],
                                 [0.00159, -0.01117, 0.02618],
                                 [-0.01168, 0.13111, 0.0046],
                                 [0.00697, 0.006, 0.06048],
                                 [0.005137, 0.0009572, -0.06682]])

        self.Ixx = [0.0470910226, 0.027885975, 0.0266173355, 0.0131822787, 0.0166774282, 0.0070053791, 0.0008162135]
        self.Iyy = [0.035959884, 0.020787492, 0.012480083, 0.009268520, 0.003746311, 0.005527552, 0.0008735012]
        self.Izz = [0.0376697645, 0.0117520941, 0.0284435520, 0.0071158268, 0.0167545726, 0.0038760715, 0.0005494148]
        self.Ixy = [-0.0061487003, -0.0001882199, -0.0039218988, -0.0001966341, -0.0001865762, 0.0001534806, 0.000128440 ]
        self.Iyz = [-0.0007808689, 0.0020767576, -0.001083893, 0.000745949, 0.0006473235, -0.0002111503, 0.0001057726 ]
        self.Ixz = [0.0001278755, -0.00030096397, 0.0002927063, 0.0003603617, 0.0001840370, -0.0004438478, 0.00018969891]
        self.inertia_mat = []

        self.link_mass = np.array([5.70044, 3.22698, 4.31272, 2.07206, 2.24665, 1.60979, 0.54218])

        self.Q_j = np.matrix([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
        # print ("Q_matrix \n \n", self.Q_j)

        self.J_i_mat = []
        self.num_links = 7
        self.M_mat = None
        self.U_ij = None
        self.Uij =[]
        self.U_ijk = None
        self.Uijk =[]
        self.joint_angles = None
        self.joint_velocities = None
        self.joint_acceleration = None

        self.baxter_robot = Baxter()
        self.fk_obj = fk(self.baxter_robot)

    def calc_inertia_mat(self):
        '''Calculating the individual inertia matrices in there own frames of reference'''
        for i in range(0,7):

            I = np.matrix([[self.Ixx[i], self.Ixy[i], self.Ixz[i]], [self.Ixy[i], self.Iyy[i], self.Iyz[i]], [self.Ixz[i], self.Iyz[i], self.Izz[i]]])
            self.inertia_mat.append(I)
        return self.inertia_mat

    def calc_J_i_matrices(self):
        '''Calculating the J_i matrix'''
        for i in range(0, 7):

            J_i = np.matrix([[(-self.Ixx[i] + self.Iyy[i] + self.Izz[i])/2, self.Ixy[i], self.Ixz[i], self.link_mass[i]*self.pos_COM[i,0]],
                              [self.Ixy[i], (self.Ixx[i] - self.Iyy[i] + self.Izz[i])/2, self.Iyz[i], self.link_mass[i]*self.pos_COM[i,1]],
                              [self.Ixz[i], self.Iyz[i], (self.Ixx[i] + self.Iyy[i] - self.Izz[i])/2, self.link_mass[i]*self.pos_COM[i,2]],
                              [self.link_mass[i]*self.pos_COM[i,0], self.link_mass[i]*self.pos_COM[i,1], self.link_mass[i]*self.pos_COM[i,2], self.link_mass[i]]
                             ])
            self.J_i_mat.append(J_i)
        return self.J_i_mat

    def calc_trans_matrices(self, joint_angles):
        '''Calculating the individual transformation matrices to needed for transformation of Inertia matrices'''
        lenn = len(joint_angles.keys())
        # print ("\n Length of joint angle list \n", lenn )
        T_0_i = [ t for t in self.fk_obj.solveIntermediateFK(joint_angles)]

        # print ("\n\nTotal Transformation matrix\n\n", T_0_i[0] )

        return T_0_i

    def calc_U_ij_mat(self, T_0_i_all):
        '''The matrix Ui j is the rate of change of points on link i relative to the base as the joint position q j changes'''

        self.U_ij = np.matrix(np.ones([self.num_links, self.num_links]))
        print(self.U_ij)
        for i in xrange(0, self.num_links):
            for j in xrange(0, self.num_links):

                if j<=i:
                    self.U_ij[i,j] = print(T_0_i_all[j-1] * self.Q_j * ( inv(T_0_i_all[j-1]) * T_0_i_all[j]))
                elif j>i:
                    self.U_ij[i,j] = 0
                self.Uij.append(self.U_ij)
        return self.U_ij

    def calc_U_ijk_mat(self, T_0_i_all):
        '''The matrix U_ijk is some random bullshit -  Interaction effect between joints '''

        for i in xrange(0, self.num_links):
            for j in xrange(0, self.num_links):
                for k in xrange(0, self.num_links):

                    if (i>=k) and (k>=j):
                        self.U_ijk = T_0_i_all[j-1]*self.Q_j*(inv(T_0_i_all[j-1])*T_0_i_all[k-1])*self.Q_j*(inv(T_0_i_all[k-1])*T_0_i_all[i])
                    elif (i>=j) and (j>=k):
                        self.U_ijk = T_0_i_all[k-1]*self.Q_j*(inv(T_0_i_all[k-1])*T_0_i_all[j-1])*self.Q_j*(inv(T_0_i_all[j-1])*T_0_i_all[i])

                    elif i<j or i<k:
                        self.U_ijk = 0

                    self.Uijk.append(self.U_ijk)
                    #print(self.Uijk)


    def calc_M_matrix(self):
        '''Calculating all the individual M_i_k elements that form the M inertia matrix

        i and k are the link number and joint number respectively
        M = n x n
        '''
        self.M_mat = np.matrix(np.ones([self.num_links, self.num_links]))

        for i in range(0,7):

            for k in range(0,7):
                j = max(i,k)
                summ = 0
                for index in range(j,self.num_links):
					#print(self.U_ij[j,k])
					#print(self.J_i_mat[j])
					summ += np.trace(self.U_ij[j,k] * self.J_i_mat[j] * np.transpose(self.U_ij[j,i]))
                self.M_mat[i, k] = summ

        return self.M_mat


    def calc_C_matrix(self):
        '''Calculating the complete Coriolis effect vector
        C = n x 1
        '''

        self.C_mat = np.matrix(np.ones([self.num_links, 1]))

        for i in range(0, self.num_links):
            sum_temp_m = 0
            for k in range(0, self.num_links):
                for m in range(0,self.num_links):

                    j = max(max(i,k), m)
                    h_ikm = 0
                    for index in range(0,j):
                        h_ikm += np.trace(self.U_ijk[j,k,m] * self.J[j] * np.transpose(self.U_ij[j,i]))

                sum_temp_m += h_ikm*self.joint_velocities[k]*self.joint_velocities[m]

            self.C_mat[i] = sum_temp_m

        return self.C_mat



    def calc_G_matrix(self):
        '''Calculating the gravity effect vector
        G = n x 1'''


        g=[0,0,-9.81,0]
        for i in xrange(0, self.num_links):
            j = i
            G =0
            for index in range(0,j):
            	G = -self.link_mass[j]*g*self.U_ij[j,i]*pos_COM[j]

        temp_G += G
            
def main():
	obj = Dynamics(Baxter)
	theta = {'s0':0, 's1':0 , 'e0':0 , 'e1':0, 'w0':0 , 'w1':0 , 'w2':0}
	#theta = deg2rad(theta)
	obj.calc_inertia_mat()
	obj.calc_J_i_matrices()
	T = obj.calc_trans_matrices(theta)
	obj.calc_U_ij_mat(T)
	obj.calc_U_ijk_mat(T)
	M_matrix = obj.calc_M_matrix()
	C_matrix = obj.calc_C_matrix()
	G_matrix = obj.calc_G_matrix()
	#print('M_matrix = ',M_matrix)
	#print('C_matrix = ',C_matrix)
	#print('G_matrix = ',G_matrix)
	
# Main 
  

if __name__ == "__main__":
  main()
  
