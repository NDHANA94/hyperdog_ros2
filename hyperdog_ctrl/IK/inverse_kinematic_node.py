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
            and np.any(ang_FR != None) and np.any(ang_FL != None) and np.any(ang_BR != None) and np.any(ang_BL != None):
            for i in range (3):
                ang_FR[i] = np.rad2deg(ang_FR[i])
                ang_FL[i] = np.rad2deg(ang_FL[i])
                ang_BR[i] = np.rad2deg(ang_BR[i])
                ang_BL[i] = np.rad2deg(ang_BL[i])
            self.joint_angs.data = [
                                ang_FR[0], ang_FR[1], ang_FR[1]+ang_FR[2],
                                ang_FL[0], ang_FL[1], ang_FL[1]+ang_FL[2],
                                ang_BR[0], ang_BR[1], ang_BR[1]+ang_BR[2],
                                ang_BL[0], ang_BL[1], ang_BL[1]+ang_BL[2]
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
