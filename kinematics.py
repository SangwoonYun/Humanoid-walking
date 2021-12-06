import numpy as np
import math

class DH():
    def __init__(self, theta=0, alpha=0, l=0, d=0, invert=1):
        self.invert = invert
        self.H = np.zeros(16).reshape(4, 4)
        self.set_param(theta, alpha, l, d)
        self.set_H()

    def set_param(self, theta, alpha, l, d):
        self.dtheta = theta  # default_theta
        self.alpha  = alpha  # link_twist_deg
        self.l      = l      # link_lenght_m
        self.d      = d      # joint+offset_m
        self._alpha = math.radians(self.alpha) # link_twist_rad
        self.cos_alpha = math.cos(self._alpha)
        self.sin_alpha = math.sin(self._alpha)
        self.set_theta(0)

    def set_theta(self, value=0):
        self.theta  = self.invert * (self.dtheta + value) # joint_angle_deg
        self._theta = math.radians(self.theta) # joint_angle_rad
        self.cos_theta = math.cos(self._theta) 
        self.sin_theta = math.sin(self._theta)

    def set_H(self):
        self.H[0,0] =  self.cos_theta
        self.H[0,1] = -self.cos_alpha * self.sin_theta
        self.H[0,2] =  self.sin_alpha * self.sin_theta
        self.H[0,3] =  self.cos_theta * self.l
        self.H[1,0] =  self.sin_theta
        self.H[1,1] =  self.cos_alpha * self.cos_theta
        self.H[1,2] = -self.sin_alpha * self.cos_theta
        self.H[1,3] =  self.sin_theta * self.l
        self.H[2,1] =  self.sin_alpha
        self.H[2,2] =  self.cos_alpha
        self.H[2,3] =  self.d
        self.H[3,3] =  1

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
        self.set_origin()
        self.H.append(DH(  0, 90,  0,  0)) # hip_pitch
        self.H.append(DH( 90,-90,  0,  0)) # hip_roll
        self.H.append(DH(-90,-90,  0,-74)) # hip_yaw
        self.H.append(DH( 90,  0, 77,  0)) # knee_pitch
        self.H.append(DH(  0, 90,  0,  0)) # ankle_pitch
        self.H.append(DH(  0,  0, 33,  0)) # ankle_roll
        self.set_skiplate()
        self.fw_kinematics()

    def set_origin(self):
        DH_ORG = DH()
        DH_ORG.H = np.array([[ 0,-1, 0, 0],
                             [ 0, 0, 1, 0], 
                             [-1, 0, 0, 0],
                             [ 0, 0, 0, 1]])
        self.H.append(DH_ORG)

    def set_skiplate(self):
        DH_SKI = DH()
        DH_SKI.H = np.array([[ 0, 0,-1, 0],
                             [ 0, 1, 0, 0], 
                             [ 1, 0, 0, 0],
                             [ 0, 0, 0, 1]])
        self.H.append(DH_SKI)

    def get_thetas(self):
        thetas = []
        for H in self.H[1:-1]:
            thetas.append(H.theta - H.dtheta)
        return thetas

    def set_thetas(self, theta):
        for idx, H in enumerate(self.H[1:-1]):
            H.set_theta(theta[idx])
            H.set_H()

    def set_theta(self, theta, idx):
        self.H[idx].set_theta(math.degrees(theta))

    def fw_kinematics(self):    # get_foot_position_T
        T = self.H[0]
        for H in self.H[1:]:
            T = T * H
        self.T = T
        return self.T
    
    def get_theta1(self):
        r12 = self.T.H[0,1]
        r13 = self.T.H[0,2]
        r22 = self.T.H[1,1]
        r23 = self.T.H[1,2]
        theta2 = self.H[2]._theta
        theta6 = self.H[6]._theta
        cos_theta2 = math.cos(theta2)
        sin_theta6 = math.sin(theta6)
        cos_theta6 = math.cos(theta6)
        templn = r12 * cos_theta6 - r13 * sin_theta6
        templd = -cos_theta2
        templ = templn / templd
        temprn = r22 * cos_theta6 - r23 * sin_theta6
        temprd = cos_theta2
        tempr = temprn / temprd
        theta1 = math.atan2(templ, tempr)
        return theta1

    def get_theta2(self):
        r32 = self.T.H[2,1]
        r33 = self.T.H[2,2]
        theta6 = self.H[6]._theta
        templ = r32 * math.cos(theta6)
        tempr = r33 * math.sin(theta6)
        theta2 = math.asin(templ - tempr)
        return theta2

    def get_theta3(self, px, py, pz):
        theta1 = self.H[1]._theta
        theta2 = self.H[2]._theta
        theta4 = self.H[4]._theta
        sin_theta1 = math.sin(theta1)
        cos_theta1 = math.cos(theta1)
        sin_theta2 = math.sin(theta2)
        cos_theta2 = math.cos(theta2)
        sin_theta4 = math.sin(theta4)
        cos_theta4 = math.cos(theta4)
        l3 = -self.H[3].d
        l4 = self.H[4].l
        l6 = self.H[6].l
        r13 = self.T.H[0,2]
        r23 = self.T.H[1,2]
        r33 = self.T.H[2,2]
        tempa1 = -px * sin_theta1 * sin_theta2
        tempa2 =  py * cos_theta1 * sin_theta2
        tempa3 = -pz * cos_theta2
        tempa41 = r13 * sin_theta1 * sin_theta2
        tempa42 = r23 * cos_theta1 * sin_theta2
        tempa43 = r33 * cos_theta2
        alpha = tempa1 + tempa2 + tempa3 - l6 * (tempa41 - tempa42 + tempa43)
        beta  = -px * cos_theta1 - py * sin_theta1 - l6 * (r13 * cos_theta1 + r23 * sin_theta1)
        templ = -l4 * sin_theta4 * alpha + (l3 + l4 * cos_theta4) * beta
        tempr =  l4 * sin_theta4 * beta  + (l3 + l4 * cos_theta4) * alpha
        theta3 = math.atan2(templ, tempr)
        return theta3

    def get_l05(self, px, py, pz):
        P = self.T.H
        l6 = self.H[6].l
        tempx = px + l6 * P[0,2]
        tempy = py + l6 * P[1,2]
        tempz = pz + l6 * P[2,2]
        l05 = (tempx**2 + tempy**2 + tempz**2)**0.5
        return l05

    def get_theta4(self, px, py, pz):
        l3  = -self.H[3].d
        l4  = self.H[4].l
        l05 = self.get_l05(px, py, pz)
        tempn = l05**2 - l3**2 - l4**2
        tempd = 2 * l3 * l4
        theta4 = math.acos(tempn / tempd)
        return theta4

    def get_theta5(self):
        r11 = self.T.H[0,0]
        r21 = self.T.H[1,0]
        r31 = self.T.H[2,0]
        theta1 = self.H[1]._theta
        theta2 = self.H[2]._theta
        sin_theta1 = math.sin(theta1)
        cos_theta1 = math.cos(theta1)
        sin_theta2 = math.sin(theta2)
        cos_theta2 = math.cos(theta2)
        templ = -r11 * sin_theta1 * sin_theta2 + r21 * cos_theta1 * sin_theta2 - r31 * cos_theta2
        tempr = r11 * cos_theta1 + r21 * sin_theta1
        theta_pitch = math.atan2(templ, tempr)
        theta5 = theta_pitch - self.H[3]._theta - self.H[4]._theta
        return theta5

    def get_theta6(self, px, py, pz):
        self.T.H[0,3] = px
        self.T.H[1,3] = py
        self.T.H[2,3] = pz
        Pi = np.linalg.inv(self.T.H)
        Pi24 = Pi[1,3]
        Pi34 = Pi[2,3]
        l6 = self.H[6].l
        l06yz = math.sqrt(Pi24**2 + Pi34**2)
        l05yz = math.sqrt(Pi24**2 + (Pi34 - l6)**2)
        templ = math.sin(Pi24)
        temprn = l06yz**2 - l05yz**2 - l6**2
        temprd = 2 * l05yz * l6
        try:
            tempr = math.acos(temprn / temprd)
        except ValueError:
            tempr = 0
        theta6 = templ * tempr
        return theta6

    def iv_kinematics(self, px, py, pz):
        theta4 = self.get_theta4(px, py, pz)
        self.set_theta(theta4, 4)
        theta6 = self.get_theta6(px, py, pz)
        self.set_theta(theta6, 6)
        theta2 = self.get_theta2()
        self.set_theta(theta2, 2)
        theta1 = self.get_theta1()
        self.set_theta(theta1, 1)
        theta3 = self.get_theta3(px, py, pz)
        self.set_theta(theta3, 3)
        theta5 = self.get_theta5()
        self.set_theta(theta5, 5)
        # change theta1 & theta3
        #temp = self.H[1]._theta
        #self.set_theta(self.H[3]._theta, 1)
        #self.set_theta(temp, 3)

if __name__ == "__main__":
    np.set_printoptions(precision=1, suppress=True)
    leg_left = LEG()
    theta = [60, 0, 0, 30, 0, 0]
    print(["{0:0.2f}".format(i) for i in theta])
    leg_left.set_thetas(theta)
    leg_left.fw_kinematics()
    print(leg_left.T, '\n')
    leg_left.iv_kinematics(leg_left.T.H[0,3], leg_left.T.H[1,3], leg_left.T.H[2,3])
    test = leg_left.get_thetas();
    print(["{0:0.2f}".format(i) for i in test])
    leg_left.set_thetas(theta)
    leg_left.fw_kinematics()
    print(leg_left.T, '\n')
    leg_left.set_thetas(theta)
    leg_left.fw_kinematics()
    print(leg_left.T, '\n')
