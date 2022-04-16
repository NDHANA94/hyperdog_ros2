import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32MultiArray


class hyperdog_gazebo_joint_cmd_Node(Node):
    def __init__(self):
        self.joint_angs = Float64MultiArray()
        self.rx_data = []
        self.prev_joint_angs = None
        super().__init__('hyperdog_gazebo_joint_cmd')
        self.sub_ = self.create_subscription(Float32MultiArray, 'hyperdog_jointController/commands', self.sub_callback, 30)
        self.pub_ = self.create_publisher(Float64MultiArray, 'gazebo_joint_controller/commands', 30)
        timer_period = 0.01
        # self.timerPub = self.create_timer(timer_period, callback =self.pub_callback1 )
        self.timerPub = self.create_timer(timer_period, self.pub_callback)
        
    def sub_callback(self, msg):
        self.rx_data = []
        for data in msg.data:
            self.rx_data.append(data*np.pi/180)
        self.joint_angs.data = self.rx_data
        self.pub_.publish(self.joint_angs)
            
    def pub_callback(self):
        pass
        



def main(args=None):
    rclpy.init(args=args)
    inv_kin = hyperdog_gazebo_joint_cmd_Node()
    rclpy.spin(inv_kin)
    inv_kin.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
