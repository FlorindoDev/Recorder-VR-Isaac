import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')       # nome del nodo
        self.subscription = self.create_subscription(
            String,         # tipo di messaggio da ricevere
            'topic',        # nome del topic (deve corrispondere)
            self.listener_callback, 
            10              # profondità coda QoS
        )
        self.subscription  # previene warning su variabile non usata

    def listener_callback(self, msg):
	    # stampa il dato ricevuto
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()