import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

RightHandIndexRotData = 0.0
RightHandIndex1Data = 0.0
RightHandIndex2Data = 0.0
RightHandIndex3Data = 0.0

class RosNode(Node):
    publishers = {} 

    def __init__(self,name):
        super().__init__(name)


    def add_publisher(self,type,name,frequency):
        pub = self.create_publisher(type,name,frequency)
        self.publishers[name] = pub

    def publish(self,type,name,data):
        pub = self.publishers[name] 
        msg = type()
        msg.data = data
        pub.publish(msg)


def create_publishers(node):
    node.add_publisher(Float64, "RightHandIndexRot" , 50)
    node.add_publisher(Float64, "RightHandIndex1" , 50)
    node.add_publisher(Float64, "RightHandIndex2" , 50)
    node.add_publisher(Float64, "RightHandIndex3" , 50)

def publish_data(node):
        global RightHandIndexRotData , RightHandIndex1Data , RightHandIndex2Data 
        global RightHandIndex3Data
        node.publish(Float64,"RightHandIndexRot" , RightHandIndexRotData)
        node.publish(Float64,"RightHandIndex1" , RightHandIndex1Data)
        node.publish(Float64,"RightHandIndex2" , RightHandIndex2Data)
        node.publish(Float64,"RightHandIndex3" , RightHandIndex3Data)
        RightHandIndexRotData = 0.0
        RightHandIndex1Data = 0.0
        RightHandIndex2Data = 0.0
        RightHandIndex3Data = 0.0

def get_data():
        global RightHandIndexRotData , RightHandIndex1Data , RightHandIndex2Data 
        global RightHandIndex3Data

        RightHandIndexRotData = 0.0   #subs with true values of joint speed 
        RightHandIndex1Data = 0.0
        RightHandIndex2Data = 0.0
        RightHandIndex3Data = 0.0

def publish_data(node):
        node.publish(Float64,"RightHandIndexRot" , RightHandIndexRotData)
        node.publish(Float64,"RightHandIndex1" , RightHandIndex1Data)
        node.publish(Float64,"RightHandIndex2" , RightHandIndex2Data)
        node.publish(Float64,"RightHandIndex3" , RightHandIndex3Data)

def main(args=None):
    rclpy.init(args=args)
    node = RosNode('JointSpeedPub')
    create_publishers(node)

    while True:
        get_data()
        publish_data(node)
    
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

