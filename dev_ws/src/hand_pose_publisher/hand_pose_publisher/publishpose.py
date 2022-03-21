from serial.serialutil import XOFF
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray 
import numpy as np

RightHandPos_X = 0
RightHandPos_Y = 0
RightHandPos_Z = 0
RightHandRot_X = 0
RightHandRot_Y = 0
RightHandRot_Z = 0

RADTODEG = float(180/np.pi)

class RosNode(Node):
    publishers = {} 

    def __init__(self,name):
        super().__init__(name)


    def add_publisher(self,type,name,frequency):
        pub = self.create_publisher(type,name,frequency)
        self.publishers[name] = pub

    def add_subscriber(self,type,name,callback,frequency):
        self.create_subscription(type,name,callback,frequency)

    def publish(self,type,name,data):
        pub = self.publishers[name] 
        msg = type()
        msg.data = data
        pub.publish(msg)

def publish_datas(msg):
    global node 
    global RightHandPos_X , RightHandPos_Y , RightHandPos_Z
    global RightHandRot_X , RightHandRot_Y , RightHandRot_Z
    data = msg.data
    pos_x = data[0] 
    pos_y = data[1] 
    pos_z = data[2] 
    rot_x = data[3] * RADTODEG
    rot_y = data[4] * RADTODEG
    rot_z = data[5] * RADTODEG
    node.publish(Float32MultiArray, "RightHand_pos" , [pos_y , pos_x , pos_z])
    node.publish(Float32MultiArray, "RightHand_rot" , [rot_x , rot_z , rot_y])


def create_subscribers(node):
    #node.add_subscriber(Float64MultiArray, "MPU0_rawdata" , None, 50)
    node.add_subscriber(Float32MultiArray, "MPU_0_filtered_pose_and_asset" , publish_datas , 50)
    #node.add_subscriber(Float64MultiArray, "MPU_1_filtered_pose_and_asset" , None, 50)
    #node.add_subscriber(Float64MultiArray, "MPU_2_filtered_pose_and_asset" , None, 50)

def create_publishers(node):
    node.add_publisher(Float32MultiArray, "RightHand_pos" , 50)
    node.add_publisher(Float32MultiArray, "RightHand_rot" , 50)


def main(args=None):
    global node 
    rclpy.init(args=args)
    node = RosNode('RightHandPoseCalc')
    create_publishers(node)
    create_subscribers(node)
    
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()