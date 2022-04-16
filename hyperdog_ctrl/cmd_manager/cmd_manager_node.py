import rclpy
from rclpy.node import Node
import numpy as np

from hyperdog_msgs.msg import JoyCtrlCmds
from hyperdog_msgs.msg import Geometry
from geometry_msgs.msg import Twist
from IK.InverseKinematics import InverseKinematics
from cmd_manager.hyperdog_variables import Body, Leg, Cmds

from std_msgs.msg import String
import threading
from threading import Thread
import logging
import time



class CmdManager_ROS():
    def __init__(self, set_msgs, send_msgs, node_name = 'cmd_manager_node'):

        super(CmdManager_ROS, self).__init__()
        # ROS parameters
        self.node = None
        self.node_name = node_name
        # -----   sub1  -----------
        self.sub1 = None
        self.sub1_name = 'hyperdog_joy_ctrl_cmd' 
        self.sub1_interface = JoyCtrlCmds  #hyperdog_msgs.msg.CtrlCmd
        self.sub1_callback = self._joy_cmd_callback
        self.sub1_queueSize = 30
        # -----   sub2  -----------
        self.sub2 = None
        self.sub2_name = 'vel_cmd'
        self.sub2_interface = Twist     #geometry_msgs.msg.Twist
        self.sub2_callback = self._sub2_callback
        self.sub2_queueSize = 10
        # -----   pub  -----------
        self.pub = None
        self.pub_name = 'hyperdog_geometry'
        self.pub_interface = Geometry   #hyperdog_msgs.msg.Geometry
        self.pub_timer_period = 0.001
        self.pub_timer = None
        self.pub_queueSize = 12
        self.pub_callback = self._pub_callback

        self.stop = True
        
        # Robot cmds
        self.cmd = set_msgs
        self.pub_msgs = send_msgs
        
        
    def _createNode(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.node_name)
        # self.node.get_logger().info('{} node was created!'.format(self.node_name))
        

    def create_sub1(self):
        self.sub1 = self.node.create_subscription(
                        self.sub1_interface, 
                        self.sub1_name, 
                        self.sub1_callback, 
                        self.sub1_queueSize)
        # self.node.get_logger().info('{} subscriber was created!'.format(self.sub1_name))
    
    def create_sub2(self):
        self.sub2 = self.node.create_subscription(
                        self.sub2_interface, 
                        self.sub2_name, 
                        self.sub2_callback, 
                        self.sub2_queueSize)
        # self.node.get_logger().info('{} subscriber was created!'.format(self.sub2_name))
    
    def create_pub(self):
        self.pub = self.node.create_publisher(
                            self.pub_interface,
                            self.pub_name,
                            self.pub_queueSize
                        )
        self.pub_timer = self.node.create_timer(self.pub_timer_period, self.pub_callback)
        # self.node.get_logger().info('{} subscriber was created!'.format(self.pub_name))


    def _joy_cmd_callback(self, msg):
        # ------------------------------------------
        self.cmd.mode.start = msg.states[0]
        if self.cmd.mode.start:
            self.cmd.mode.walk = msg.states[1]
            self.cmd.mode.side_walk_mode = msg.states[2]
            self.cmd.mode.gait_type = msg.gait_type
            # ------------------------------------------
            self.cmd.body.height = msg.pose.position.z
            self.cmd.body.slant[0] = msg.pose.position.x
            self.cmd.body.slant[1] = msg.pose.position.y
            # ------------------------------------------
            self.cmd.body.roll = msg.pose.orientation.x
            self.cmd.body.pitch = msg.pose.orientation.y
            self.cmd.body.yaw = msg.pose.orientation.z
            # ------------------------------------------
            
            self.cmd.gait.step_len[0] = msg.gait_step.x
            self.cmd.gait.step_len[1] = msg.gait_step.y
            self.cmd.gait.swing_step_h = msg.gait_step.z

    def _sub2_callback(self, msg):
        self.cmd.gait.cycle_time = msg.linear.x
        self.cmd.gait.swing_time = msg.angular.x
    

    def _pub_callback(self):
        msg = Geometry()
        
        msg.fr.x= self.pub_msgs[0].FR.pose.cur_coord[0]
        msg.fr.y= self.pub_msgs[0].FR.pose.cur_coord[1]
        msg.fr.z= self.pub_msgs[0].FR.pose.cur_coord[2]
        msg.fl.x= self.pub_msgs[0].FL.pose.cur_coord[0]
        msg.fl.y= self.pub_msgs[0].FL.pose.cur_coord[1]
        msg.fl.z= self.pub_msgs[0].FL.pose.cur_coord[2]
        msg.br.x= self.pub_msgs[0].BR.pose.cur_coord[0]
        msg.br.y= self.pub_msgs[0].BR.pose.cur_coord[1]
        msg.br.z= self.pub_msgs[0].BR.pose.cur_coord[2]
        msg.bl.x= self.pub_msgs[0].BL.pose.cur_coord[0]
        msg.bl.y= self.pub_msgs[0].BL.pose.cur_coord[1]
        msg.bl.z= self.pub_msgs[0].BL.pose.cur_coord[2]
        
        msg.euler_ang.x = np.deg2rad(self.pub_msgs[1].roll)
        msg.euler_ang.y = np.deg2rad(self.pub_msgs[1].pitch)
        msg.euler_ang.z = np.deg2rad(self.pub_msgs[1].yaw)

        self.pub.publish(msg)
        
        # self.node.get_logger().info('Publishing message')
            
    
    def get_numOf_threads(self):
        return threading.active_count()

    def start(self):
        
        self._createNode()
        self.create_sub1()
        self.create_sub2()
        self.create_pub()
        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()
        self.stop = False
        
        

    def stop(self):
        self.stop = True

    def run(self):
        pass
        



# def main(args=None):
#     print('starting')
#     cmd_manager = CmdManager_ROS()
#     thread1 = Thread(target=cmd_manager.start)
#     thread1.start()

#     while 1:
#         # print("tnumber of threads in background: {}".format(thread1.get_numOf_threads()))
#         print("current thread: {}".format(threading.current_thread().name))
#         time.sleep(0.1)
#         print(cmd.mode.start)


# if __name__ == '__main__':
#     main()