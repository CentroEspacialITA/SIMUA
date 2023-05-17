import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from environment import env

class SenderNode(Node):
    
    def __init__(self):
        
        rosDomainID=env['protocolKey']
        senderNodeName=env['dataStreamer']
        topicName='chatter'
        
        super().__init__(senderNodeName)
        self.publisher_=self.create_publisher(String,topicName,rosDomainID)
    
    def send_message(self):
        msg=String()
        msg.data='Hello ROS :)'
        self.publisher_.publish(msg)

    def main(args=None):
        
        rclpy.init(args=args)
        sender_node = SenderNode()
        
        while rclpy.ok():
            sender_node.send_message()
            rclpy.spin_once(sender_node)

        sender_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()