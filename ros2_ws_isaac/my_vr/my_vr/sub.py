import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from message_filters import Subscriber, ApproximateTimeSynchronizer
import csv
import os
import numpy as np
from visual_kinematics.RobotSerial import RobotSerial
from visual_kinematics.Frame import Frame
from scipy.spatial.transform import Rotation as R
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3


def kuka_iiwa_model():
    # Definizione dei parametri DH approssimati per KUKA LBR iiwa 7 R800
    links = [
        RevoluteDH(d=0.34, a=0, alpha=-np.pi/2),
        RevoluteDH(d=0,    a=0, alpha=np.pi/2),
        RevoluteDH(d=0.4,  a=0, alpha=-np.pi/2),
        RevoluteDH(d=0,    a=0, alpha=np.pi/2),
        RevoluteDH(d=0.4,  a=0, alpha=-np.pi/2),
        RevoluteDH(d=0,    a=0, alpha=np.pi/2),
        RevoluteDH(d=0.126,a=0, alpha=0),
    ]
    robot = DHRobot(links, name='KUKA_LBR_iiwa')
    return robot

def cinematic(x,y,z,o):

    # 1. Crea modello robot
    robot = kuka_iiwa_model()

    # 2. Costruisci matrice di rotazione dal quaternione
    rotation_matrix = R.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
    
    # 3. Costruisci SE3 (pose desiderata)
    T_target = SE3.Rt(rotation_matrix, [x, y, z])

    # 4. Calcolo IK numerico (Levenberg-Marquardt)
    sol = robot.ikine_LM(T_target)  # puoi anche provare .ikine_min()

    # 6. Verifica se la soluzione è raggiungibile
    if sol.success:
      

        # 7. Esportazione dei dati in CSV
        joint_positions = sol.q
         # CSV file setup
        log_dir = os.path.expanduser('~/Rec')
        os.makedirs(log_dir, exist_ok=True)
        csv_filename = os.path.join(log_dir, 'joint_position.csv')

        first_run = not os.path.exists(csv_filename)
        csv_file = open(csv_filename, 'a', newline='')
        csv_writer = csv.writer(csv_file)

        if first_run:
            # write header only once (without timestamp)
            csv_writer.writerow([
                'J1', 'J2', 'J3',
                'J4', 'J5', 'J6', 'J7'
            ])
            csv_file.flush()
        
        csv_writer.writerow([
            f"{joint_positions[0]}", f"{joint_positions[1]}", f"{joint_positions[2]}", f"{joint_positions[3]}", f"{joint_positions[4]}", f"{joint_positions[5]}", f"{joint_positions[6]}"
        ])
        return True

    else:
        print("La posizione desiderata non è raggiungibile.")
        return False


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

        #coordinate di isaac sono diverse da quelle del visore VR
        x_robot = p.z
        y_robot = -p.x
        z_robot = p.y
        # write row without timestamp
        if(cinematic(x_robot,y_robot,z_robot,o)):
            self.csv_writer.writerow([
                f"{p.x:.6f}", f"{p.y:.6f}", f"{p.z:.6f}",
                f"{o.x:.6f}", f"{o.y:.6f}", f"{o.z:.6f}", f"{o.w:.6f}",
                f"{grip:.6f}"
            ])
        else:
            self.get_logger().info("La posizione desiderata non è raggiungibile.")
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