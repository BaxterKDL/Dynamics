import numpy as np
import random
from ForwardKinematics.FkSolver import FKSolver

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

        self.J_i_mat = []

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

    def calc_U_ij_mat(self):
        '''The matrix Ui j is the rate of change of points on link i relative to the base as the joint position q j changes'''