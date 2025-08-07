import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from message_filters import Subscriber, ApproximateTimeSynchronizer
import csv
import os
import numpy as np
from visual_kinematics.RobotSerial import RobotSerial
from visual_kinematics.Frame import Frame
from scipy.spatial.transform import Rotation as R
from roboticstoolbox import DHRobot, RevoluteDH, ERobot
from spatialmath import SE3

def create_csv_file(name):
    log_dir = os.path.expanduser('~/Rec')
    os.makedirs(log_dir, exist_ok=True)
    csv_filename = os.path.join(log_dir, name)

    first_run = not os.path.exists(csv_filename)
    csv_file = open(csv_filename, 'a', newline='')
    csv_writer = csv.writer(csv_file)
    return first_run, csv_file, csv_writer


def initColumnCSV(csv_file, csv_writer, columns):
    # write header only once (without timestamp)
    csv_writer.writerow(columns)
    csv_file.flush() #coordinate di isaac sono diverse da quelle del visore VRile.flush()


#coordinate di isaac sono diverse da quelle del visore VR
def transform_input(x_in, y_in, z_in):
        # Da frame Isaac Sim a frame robot
        x_robot = x_in          # Isaac X → visore X
        y_robot = -z_in         # Isaac Y → visore -Z
        z_robot = y_in          # Isaac Z → visore Y
        return x_robot, y_robot, z_robot


def transform_orientation(quat):
    # Quaternione → matrice
    r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
    R_vr = r.as_matrix()

    # Matrice di trasformazione tra VR e Robot (corretta per segni)
    # Da VR: X, Y, Z → A Robot: X, -Z, Y
    R_convert = np.array([
        [1,  0,  0],
        [0,  0,  1],
        [0, -1,  0]
    ])
    
    # Matrice di trasformazione aggiornata:
    # X_robot = -X_vr (inverte destra/sinistra)
    # Y_robot =  Z_vr
    # Z_robot = -Y_vr
    #R_convert = np.array([
    #	[1,  0,  0],
    #	[0,  0, -1],
    #	[0,  1,  0]
    #])

    # Applica cambio di base
    R_robot = R_convert @ R_vr
    
     # 3. Ruota attorno a Z (robot) di 180° per correggere destra/sinistra
    R_fix = R.from_euler('z', 180, degrees=True).as_matrix()
    R_final = R_fix @ R_robot

    # Torna a quaternione
    quat_robot = R.from_matrix(R_final).as_quat()  # [x, y, z, w]
    return quat_robot

def kuka_iiwa_model_DH():
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

def kuka_iiwa_model_URDF():
    robot = ERobot.URDF("/home/francesco/Desktop/tirocinio/ros2_ws/urdf/urdf/lbr_iiwa7_r800.urdf")
    return robot
    

def cinematic(self,x,y,z,o):

    # 1. Crea modello robot
    robot = kuka_iiwa_model_URDF()

    # 2. Costruisci matrice di rotazione dal quaternione e normalizza
    #quat = np.array([o.x, o.y, o.z, o.w])
    #quat = quat / np.linalg.norm(quat)
    #rotation_matrix = R.from_quat(quat).as_matrix()
    quat_robot = transform_orientation(o)
    rotation_matrix = R.from_quat(quat_robot).as_matrix()

    
    # 3. Costruisci SE3 (pose desiderata)
    T_target = SE3.Rt(rotation_matrix, [x, y, z])

    # 4. Calcolo IK numerico (Levenberg-Marquardt)
    # Punto iniziale coerente (q0)
    self.last_q = getattr(self, 'last_q', np.zeros(7))
    sol = robot.ikine_LM(T_target, q0=self.last_q)

    # 5. Verifica se la soluzione è raggiungibile
    if sol.success:
      

        # 7. Esportazione dei dati in CSV
        joint_positions = sol.q

        self.joint_msg = JointState()
        self.joint_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_msg.name = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'joint6', 'joint7'
        ]
        self.joint_msg.position = [float(j) for j in joint_positions]

        
        self.get_logger().info(
            f"Joint values: {', '.join([f'{j:.3f}' for j in joint_positions])}"
        )
        
        T = robot.fkine(joint_positions)
        
        xyz_result = T.t  # oppure T.t.tolist() se vuoi in forma di lista
        self.get_logger().info(f"Posizione effettiva (FK): {xyz_result}")
        
        return True

    else:
        print("La posizione desiderata non è raggiungibile.")
        return False

    

class VRDataLogger(Node):
    def __init__(self):
        super().__init__('vr_data_logger')

        # CSV file setup
        first_run , self.csv_file_vr , self.csv_writer_vr = create_csv_file('vr_controller_data.csv') 

        col = ['hand_x', 'hand_y', 'hand_z',
                'ori_x', 'ori_y', 'ori_z', 'ori_w',
                'grip_state'
            ]
        
        if first_run:
            initColumnCSV(self.csv_file_vr,self.csv_writer_vr,col)


        # CSV file setup
        first_run,self.csv_file_joint,self.csv_writer_joint  = create_csv_file('joint_position.csv')

        col = [
                'J1', 'J2', 'J3',
                'J4', 'J5', 'J6', 'J7'
            ]

        if first_run:
            initColumnCSV(self.csv_file_joint,self.csv_writer_joint,col)
        

        # message_filters subscribers
        pose_sub = Subscriber(self, PoseStamped, 'vr/controller_pose')
        grip_sub = Subscriber(self, Float32, 'vr/trigger_pressure')
        self.publisher_ = self.create_publisher(JointState, '/joint_command', 10)

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

        x,y,z = transform_input(p.x,p.y,p.z)
        # write row without timestamp
        if(cinematic(self, x,y,z ,o)):
            self.csv_writer_vr.writerow([
                f"{p.x:.6f}", f"{p.y:.6f}", f"{p.z:.6f}",
                f"{o.x:.6f}", f"{o.y:.6f}", f"{o.z:.6f}", f"{o.w:.6f}",
                f"{grip:.6f}"
            ])
            
            self.joint_msg.name.append("left_inner_finger_joint")
            self.joint_msg.name.append("right_inner_finger_joint")
            self.joint_msg.position.append(float(grip))
            self.joint_msg.position.append(float(grip))
            self.publisher_.publish(self.joint_msg)
        
        else:
            self.get_logger().info("La posizione desiderata non è raggiungibile.")
        self.csv_file_vr.flush()

    def destroy_node(self):
        # close CSV
        try:
            self.csv_file_vr.close()
            self.csv_file_joint.close()
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
