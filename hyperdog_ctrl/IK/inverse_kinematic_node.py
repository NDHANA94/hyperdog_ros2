# __________________________________________________________________________________
# MIT License                                                                       |
#                                                                                   |
# Copyright (c) 2024 W.M. Nipun Dhananjaya Weerakkodi                               |
#                                                                                   | 
# Permission is hereby granted, free of charge, to any person obtaining a copy      |
# of this software and associated documentation files (the "Software"), to deal     |
# in the Software without restriction, including without limitation the rights      |
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell         |
# copies of the Software, and to permit persons to whom the Software is             |
# furnished to do so, subject to the following conditions:                          |
#                                                                                   |
# The above copyright notice and this permission notice shall be included in all    |
# copies or substantial portions of the Software.                                   |
#                                                                                   |
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR        |
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,          |
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE       |
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER            |
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,     |
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE     |
# SOFTWARE.                                                                         |
# __________________________________________________________________________________|

import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32MultiArray
from hyperdog_msgs.msg import Geometry
from IK.InverseKinematics import InverseKinematics






class InvKin_Node(Node):
    def __init__(self):
        self.IK =  InverseKinematics()
        self.joint_angs = Float32MultiArray()
        self.prev_joint_angs = None
        super().__init__('IK_node')
        self.sub_ = self.create_subscription(Geometry, 'hyperdog_geometry', self.sub_callback, 30)
        self.pub2STM = self.create_publisher(Float32MultiArray, 'hyperdog_jointController/commands', 30)
        timer_period = 0.02
        # self.timerPub = self.create_timer(timer_period, callback =self.pub_callback1 )
        self.timerPub = self.create_timer(timer_period, self.pub_callback)
        


    def sub_callback(self, msg):
        eulerAng = np.array([msg.euler_ang.x, msg.euler_ang.y, msg.euler_ang.z])
        fr_coord = np.array([msg.fr.x, msg.fr.y, msg.fr.z])
        fl_coord = np.array([msg.fl.x, msg.fl.y, msg.fl.z])
        br_coord = np.array([msg.br.x, msg.br.y, msg.br.z])
        bl_coord = np.array([msg.bl.x, msg.bl.y, msg.bl.z])

        ang_FR = self.IK.get_FR_joint_angles(fr_coord, eulerAng)
        ang_FL = self.IK.get_FL_joint_angles(fl_coord, eulerAng)
        ang_BR = self.IK.get_BR_joint_angles(br_coord, eulerAng)
        ang_BL = self.IK.get_BL_joint_angles(bl_coord, eulerAng)
        # self.get_logger().info('singularity: {}!'.format(self.IK.singularity))
        if not np.any(self.IK.singularity) \
            and ang_FR is not None and ang_FL is not None and ang_BR is not None and ang_BL is not None:

            ang_FR_deg = np.rad2deg(ang_FR)
            ang_FL_deg = np.rad2deg(ang_FL)
            ang_BR_deg = np.rad2deg(ang_BR)
            ang_BL_deg = np.rad2deg(ang_BL)

            self.joint_angs.data = [
                                ang_FR_deg[0], ang_FR_deg[1], ang_FR_deg[1]+ang_FR_deg[2],
                                ang_FL_deg[0], ang_FL_deg[1], ang_FL_deg[1]+ang_FL_deg[2],
                                ang_BR_deg[0], ang_BR_deg[1], ang_BR_deg[1]+ang_BR_deg[2],
                                ang_BL_deg[0], ang_BL_deg[1], ang_BL_deg[1]+ang_BL_deg[2]
                                ] 
            self.prev_joint_angs = self.joint_angs.data
            # self.pub2STM.publish(self.joint_angs) 
        elif not self.prev_joint_angs == None:
            self.joint_angs.data = self.prev_joint_angs
        
            


    def pub_callback(self):
        if np.any(self.joint_angs.data) != None:
            pass
            self.pub2STM.publish(self.joint_angs)    
    

def main(args=None):
    rclpy.init(args=args)
    inv_kin = InvKin_Node()
    rclpy.spin(inv_kin)
    inv_kin.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
