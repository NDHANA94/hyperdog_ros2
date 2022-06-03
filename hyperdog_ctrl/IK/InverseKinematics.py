from ntpath import join
import numpy as np


class LegStates():
    prev_coord = None
    now_coord  = None
    prev_angles = None
    now_angles = None
    in_singularity = None

class Legs():
    FR = LegStates()
    FL = LegStates()
    BR = LegStates()
    BL = LegStates()


class InverseKinematics():
    def __init__(self):
        self.L1 = 104
        self.L2 = 150
        self.L3 = 150
        self.BODY_LENGTH = 300
        self.BODY_WIDTH  = 172
        self.MIN_ANG_L2L3 = 30
        self.MAX_ANG_L2L3 = 170
        self.MIN_LEG_R   = np.sqrt(self.L1**2 + (self.L2**2 + self.L3**2 - 2*self.L2*self.L3*np.cos(np.deg2rad(self.MIN_ANG_L2L3))))
        self.MAX_LEG_R   = np.sqrt(self.L1**2 + (self.L2**2 + self.L3**2 - 2*self.L2*self.L3*np.cos(np.deg2rad(self.MAX_ANG_L2L3))))

        self.M_R = np.array([1, 1, 1])
        self.M_L = np.array([1, -1, 1])
        self.M_F = np.array([1, 1, 1])
        self.M_B = np.array([-1, 1, 1])
        self.BODY_SCALE = np.array([self.BODY_LENGTH/2, self.BODY_WIDTH/2, 0])

        self.legState = Legs()

        self.singularity = [0,0,0,0]

    def rotMat(self, eularAng):
        """
            eularAng: np array [roll, pitch, yaw]
            return : rotation matrix
        """
        M11 = np.cos(eularAng[1])*np.cos(eularAng[2])
        M12 = np.sin(eularAng[0])*np.sin(eularAng[1])*np.cos(eularAng[2]) - np.cos(eularAng[0])*np.sin(eularAng[2])
        M13 = np.cos(eularAng[0])*np.sin(eularAng[1])*np.cos(eularAng[2]) + np.sin(eularAng[0])*np.sin(eularAng[2])

        M21 = np.cos(eularAng[1])*np.sin(eularAng[2])
        M22 = np.sin(eularAng[0])*np.sin(eularAng[1])*np.sin(eularAng[2]) + np.cos(eularAng[0])*np.cos(eularAng[2])
        M23 = np.cos(eularAng[0])*np.sin(eularAng[1])*np.sin(eularAng[2]) - np.sin(eularAng[0])*np.cos(eularAng[2])
        
        M31 = -np.sin(eularAng[1])
        M32 = np.sin(eularAng[0])*np.cos(eularAng[1])
        M33 = np.cos(eularAng[0])*np.cos(eularAng[1])
        rotMat = np.array([ [M11, M12, M13], [M21, M22, M23], [M31, M32, M33] ])
        return rotMat

    def get_joint_angles(self, coord):
        """
            coord: np array with desired leg coordinate -> np.array([x, y, z])
            return: np array with joint angles -> np.array([th0, th1, th2])
        """
        joint_ang = np.zeros([3])
        if self.is_singularity(coord):
            pass
        else:
            r_yz = np.sqrt(coord[1]**2 + coord[2]**2) 
            alpha = np.arccos(coord[1]/r_yz)
            th0 = np.arccos(self.L1/r_yz) - alpha #phi1 - phi2
            if not np.isnan(th0):
                joint_ang[0] = th0

            a = 2*self.L2*r_yz*np.sin(joint_ang[0]+ alpha)
            b = 2*self.L2*coord[0]
            c = coord[0]**2 + self.L2**2 - self.L3**2 + (a/(2*self.L2))**2
            d = np.sqrt(a**2 + b**2)
            beta = np.arccos(a/d)

            if (coord[0] < 0 ):
                beta = -beta
            if a/d > 1 or a/d < -1 or c/d >1 or c/d < -1:
                pass
            else:
                th1 = beta + np.arcsin(c/d)
                if not np.isnan(th1):
                    joint_ang[1] = th1
                
                th2 = np.arccos((coord[0] + self.L2*np.cos(joint_ang[1]))/self.L3)
                if not np.isnan(th2):
                    joint_ang[2] = th2
        return joint_ang

    def get_FR_joint_angles(self, coord, eularAng):
        # rotate around midle of the body
        translate_FR = self.BODY_SCALE*self.M_F*self.M_R
        coord_ = (np.dot((coord * self.M_R + translate_FR), self.rotMat(eularAng)) - translate_FR) * self.M_R
        # check singularity of the legs
        if self.is_singularity(coord_):
                self.singularity[0] = True
        else:
            self.singularity[0] = False
        #  check if any legs is in singularity
        if any(self.singularity):
            self.legState.FR.now_angles = self.legState.FR.prev_angles
        # if no singularities, get new joint angles
        else:
            self.legState.FR.now_angles = self.get_joint_angles(coord_)
            self.legState.FR.prev_angles = self.legState.FR.now_angles
            self.singularity[0] = False
        return self.legState.FR.now_angles

    def get_FL_joint_angles(self, coord, eularAng):
        translate_FL = self.BODY_SCALE*self.M_F*self.M_L
        coord_ = (np.dot((coord * self.M_L + translate_FL), self.rotMat(eularAng)) - translate_FL) * self.M_L
        # check singularity of the legs
        if self.is_singularity(coord_):
                self.singularity[1] = True
        else:
            self.singularity[1] = False
        if any(self.singularity):
            self.legState.FL.now_angles = self.legState.FL.prev_angles
        # if no singularities, get new joint angles
        else:
            self.legState.FL.now_angles = self.get_joint_angles(coord_)
            self.legState.FL.prev_angles = self.legState.FL.now_angles
            self.singularity[1] = False
        return self.legState.FL.now_angles
     

    def get_BR_joint_angles(self, coord, eularAng):
        translate_BR = self.BODY_SCALE*self.M_B*self.M_R
        coord_ = (np.dot((coord * self.M_R + translate_BR), self.rotMat(eularAng)) - translate_BR) * self.M_R
        # check singularity of the legs
        if self.is_singularity(coord_):
                self.singularity[2] = True
        else:
            self.singularity[2] = False
        if any(self.singularity):
            self.legState.BR.now_angles = self.legState.BR.prev_angles
        # if no singularities, get new joint angles
        else:
            self.legState.BR.now_angles = self.get_joint_angles(coord_)
            self.legState.BR.prev_angles = self.legState.BR.now_angles
            self.singularity[2] = False
        return self.legState.BR.now_angles

    def get_BL_joint_angles(self, coord, eularAng):
        translate_FL = self.BODY_SCALE*self.M_B*self.M_L
        coord_ = (np.dot((coord * self.M_L + translate_FL), self.rotMat(eularAng)) - translate_FL) * self.M_L
        # check singularity of the legs
        if self.is_singularity(coord_):
                self.singularity[3] = True
        else:
            self.singularity[3] = False
        if any(self.singularity):
            self.legState.BL.now_angles = self.legState.BL.prev_angles
        # if no singularities, get new joint angles
        else:
            self.legState.BL.now_angles = self.get_joint_angles(coord_)
            self.legState.BL.prev_angles = self.legState.BL.now_angles
            self.singularity[3] = False
        return self.legState.BL.now_angles

    def is_singularity(self, coord):
        r = np.sqrt(coord[0]**2 + coord[1]**2 + coord[2]**2)
        if r >= self.MIN_LEG_R and r <= self.MAX_LEG_R:
            return False
        else:
            return True