import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from message_filters import Subscriber, ApproximateTimeSynchronizer
import csv
import os

class VRDataLogger(Node):
    def __init__(self):
        super().__init__('vr_data_logger')

        # CSV file setup
        log_dir = os.path.expanduser('~/Rec')
        os.makedirs(log_dir, exist_ok=True)
        self.csv_filename = os.path.join(log_dir, 'vr_controller_data.csv')

        first_run = not os.path.exists(self.csv_filename)
        self.csv_file = open(self.csv_filename, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        if first_run:
            # write header only once (without timestamp)
            self.csv_writer.writerow([
                'hand_x', 'hand_y', 'hand_z',
                'ori_x', 'ori_y', 'ori_z', 'ori_w',
                'grip_state'
            ])
            self.csv_file.flush()

        # message_filters subscribers
        pose_sub = Subscriber(self, PoseStamped, 'vr/controller_pose')
        grip_sub = Subscriber(self, Float32, 'vr/trigger_pressure')

        # Synchronize by header timestamp / arrival time (allow headerless)
        sync = ApproximateTimeSynchronizer(
            [pose_sub, grip_sub],
            queue_size=10,
            slop=0.01,
            allow_headerless=True
        )
        sync.registerCallback(self.synced_callback)

        self.get_logger().info("VRDataLogger: pronti a ricevere pose e pressione trigger sincronizzate")

    def synced_callback(self, pose_msg: PoseStamped, grip_msg: Float32):
        # extract pose
        p = pose_msg.pose.position
        o = pose_msg.pose.orientation

        # extract grip
        grip = grip_msg.data

        # log to console without timestamp
        self.get_logger().info(
            f"pos=({p.x:.3f},{p.y:.3f},{p.z:.3f}) | "
            f"ori=({o.x:.3f},{o.y:.3f},{o.z:.3f},{o.w:.3f}) | grip={grip:.3f}"
        )

        # write row without timestamp
        self.csv_writer.writerow([
            f"{p.x:.6f}", f"{p.y:.6f}", f"{p.z:.6f}",
            f"{o.x:.6f}", f"{o.y:.6f}", f"{o.z:.6f}", f"{o.w:.6f}",
            f"{grip:.6f}"
        ])
        self.csv_file.flush()

    def destroy_node(self):
        # close CSV
        try:
            self.csv_file.close()
            self.get_logger().info(f"File '{self.csv_filename}' chiuso.")
        except Exception as e:
            self.get_logger().error(f"Errore chiusura CSV: {e}")
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VRDataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
