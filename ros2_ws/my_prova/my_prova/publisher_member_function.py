import rclpy                      
from rclpy.node import Node        
from std_msgs.msg import String    

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')   # nome del nodo
        self.publisher = self.create_publisher(
            String,      # tipo di messaggio da pubblicare
            'topic',     # nome del topic
            10           # profondità coda (QoS)
        )
        timer_period = 0.5  # intervallo in secondi
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)          # invia il messaggio
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)                       # inizializza ROS 2
    minimal_publisher = MinimalPublisher()      # crea il nodo
    rclpy.spin(minimal_publisher)               # gestisce i callback
    minimal_publisher.destroy_node()      # distrugge il nodo (pulizia)
    rclpy.shutdown()                            # chiude ROS 2

if __name__ == '__main__':
    main()