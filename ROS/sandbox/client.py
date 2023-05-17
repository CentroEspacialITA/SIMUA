import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from environment import env

class ListenerNode(Node):
    
    def __init__(self):
        
        rosDomainID=env['protocolKey']
        receiverNodeName=env['dataReceiver']
        topicName='chatter'
        
        super().__init__(receiverNodeName)
        self.subscription_=self.create_subscription(String,topicName,self.message_callback,rosDomainID)
    
    def message_callback(self, msg):
        self.get_logger().info('Received message: "%s"' % msg.data)

    def main(args=None):
        
        rclpy.init(args=args)
        listener_node = ListenerNode()
        rclpy.spin(listener_node)
        
        listener_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()