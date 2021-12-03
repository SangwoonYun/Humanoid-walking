import numpy as np
import math

class DH():
    def __init__(self, theta=0, alpha=0, l=0, d=0):
        self.H = np.zeros(16).reshape(4, 4)
        self.set_param(theta, alpha, l, d)
        self.set_H()

    def set_param(self, theta=0, alpha=0, l=0, d=0):
        self.theta = theta
        self.alpha = alpha
        self.l     = l
        self.d     = d
        self._theta = math.radians(self.theta)
        self._alpha = math.radians(self.alpha)
        self.cos_theta = math.cos(self._theta)
        self.sin_theta = math.sin(self._theta)
        self.cos_alpha = math.cos(self._alpha)
        self.sin_alpha = math.sin(self._alpha)

    def set_H(self):
        self.H[0,0] =  self.cos_theta
        self.H[0,1] = -self.cos_alpha * self.sin_theta
        self.H[0,2] =  self.sin_alpha * self.sin_theta
        self.H[0,3] =  self.cos_theta * self._alpha
        self.H[1,0] =  self.sin_theta
        self.H[1,1] =  self.cos_alpha * self.cos_theta
        self.H[1,2] = -self.sin_alpha * self.cos_theta
        self.H[1,3] =  self.sin_theta * self._alpha
        self.H[2,1] =  self.sin_alpha
        self.H[2,2] =  self.cos_alpha
        self.H[2,3] =  self.d
        self.H[3,3] =  self.l

    def __mul__(self, other):
        DH_DOT = DH()
        DH_DOT.H = self.H.dot(other.H)
        return DH_DOT

    def __str__(self):
        return str(self.H)

    def __repr__(self):
        return self.H

class LEG(): 
    def __init__(self):
        self.H = []
        self.H.append(DH(-90, 90,  0, 0))
        self.H.append(DH(  0,-90,  0, 0))
        self.H.append(DH(  0,  0, 77, 0))
        self.H.append(DH(  0,  0, 77, 0))
        self.H.append(DH(  0,  0,  0, 0))
        self.H.append(DH(  0, 90,  0, 0))

    def get_T(self):
        T = self.H[0]
        for H in self.H[1:]:
            T = T * H
        return T

if __name__ == "__main__":
    leg_left = LEG()

    test = leg_left.get_T()
    print(test)
