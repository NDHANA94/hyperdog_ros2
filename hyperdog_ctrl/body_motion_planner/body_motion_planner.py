import numpy as np
from cmd_manager.hyperdog_variables import Cmds, Body, Leg
import time
# cmd = Cmds()
# leg = Leg()
# body = Body()

class BodyMotionPlanner():
    def __init__(self , cmd, leg, body, gait_planner):
        self.cmd = cmd
        self.body = body
        self.leg = leg
        self.gait = gait_planner

        self.prev_slant = self.cmd.body.slant

        self.__L1 = self.leg.physical._L1
        self.__L2 = self.leg.physical._L2
        self.__L3 = self.leg.physical._L3

        self.cmd.leg.foot_zero_pnt[:,1] = self.__L1

    def set_init_pose(self):
        self.body.roll = 0
        self.body.pitch = 0
        self.body.yaw = 0

        self.cmd.leg.foot_zero_pnt[:,2] = np.array(self.cmd.body.height)
        self.leg.FR.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[0,:])
        self.leg.FL.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[1,:])
        self.leg.BR.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[2,:])
        self.leg.BL.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[3,:])
        return True

    def change_height(self):
        self.leg.FR.pose.cur_coord[2] = self.body.height[:]
        self.leg.FL.pose.cur_coord[2] = self.body.height[:]
        self.leg.BL.pose.cur_coord[2] = self.body.height[:]
        self.leg.BR.pose.cur_coord[2] = self.body.height[:]
        return True
    
    def run(self):
        while True:
            self.cmd.leg.foot_zero_pnt[:,2] = np.array(self.cmd.body.height) 
            """ uncomment below 2 lines to activate slant from joystick"""
            self.cmd.leg.foot_zero_pnt[:,1] = self.__L1
            # self.cmd.leg.foot_zero_pnt[::2,:2] = np.array([0,self.__L1]) + self.cmd.body.slant[:2]
            # self.cmd.leg.foot_zero_pnt[1::2,:2] = np.array([0,self.__L1]) + self.cmd.body.slant[:2]*np.array([1,-1])
            self.body.roll = self.cmd.body.roll
            self.body.pitch = self.cmd.body.pitch
            self.body.yaw = self.cmd.body.yaw
            # if np.any(self.cmd.body.slant != self.prev_slant):
            #     self.leg.FR.pose.cur_coord[:2] = 
            self.leg.FR.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[0,:]) + self.gait.FR_traj[:] + self.body.ZMP_handler[0,:]*np.array([0,1,0]) #+ self.cmd.body.slant[:]*np.array([1,1,0])
            self.leg.FL.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[1,:]) + self.gait.FL_traj[:] + self.body.ZMP_handler[1,:]*np.array([0,1,0]) #+ self.cmd.body.slant[:]*np.array([1,-1,0])
            self.leg.BR.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[2,:]) + self.gait.BR_traj[:] + self.body.ZMP_handler[2,:]*np.array([0,1,0])#+ self.cmd.body.slant[:]*np.array([1,1,0])
            self.leg.BL.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[3,:]) + self.gait.BL_traj[:] + self.body.ZMP_handler[3,:]*np.array([0,1,0])#+ self.cmd.body.slant[:]*np.array([1,-1,0])
            
            time.sleep(0.0002)
        # print(self.leg.FR.pose.cur_coord[:]) 



# leg.FR.gait.displacement 
# body.roll
