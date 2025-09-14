import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from message_filters import Subscriber, ApproximateTimeSynchronizer
import csv
import os
import numpy as np
from visual_kinematics.RobotSerial import RobotSerial
from visual_kinematics.Frame import Frame
from scipy.spatial.transform import Rotation as R
from roboticstoolbox import DHRobot, RevoluteDH, ERobot
from spatialmath import SE3

def transform_input(x_in, y_in, z_in):
        # Da frame Isaac Sim a frame robot
        #x_robot = z_in          # Isaac Z → robot X
        #y_robot = y_in         # Isaac X → robot -Y
        #z_robot = x_in          # Isaac Y → robot Z
        x_robot = x_in          # Isaac Z → robot X
        y_robot = -z_in         # Isaac X → robot -Y
        z_robot = y_in          # Isaac Y → robot Z
        return x_robot, y_robot, z_robot


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
    robot = ERobot.URDF("/home/francesco/Desktop/tirocinio/ros2_ws/urdf/urdf/lbr_iiwa7_r800.urdf")
    return robot

def cinematic(self,x,y,z,o):

    # 1. Crea modello robot
    robot = kuka_iiwa_model()

    # 2. Costruisci matrice di rotazione dal quaternione
    quat = np.array([o.x, o.y, o.z, o.w])
    quat = quat / np.linalg.norm(quat)
    rotation_matrix = R.from_quat(quat).as_matrix()
    # 3. Costruisci SE3 (pose desiderata)
    T_target = SE3.Rt(rotation_matrix, [x, y, z])

    # 4. Calcolo IK numerico (Levenberg-Marquardt)
    #sol = robot.ikine_LM(T_target)  # puoi anche provare .ikine_min()

    # 4. Punto iniziale coerente (q0)
    self.last_q = getattr(self, 'last_q', np.zeros(7))
    sol = robot.ikine_LM(T_target, q0=self.last_q)

    # 6. Verifica se la soluzione è raggiungibile
    if sol.success:
      

        # 7. Esportazione dei dati in CSV
        joint_positions = sol.q

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'joint6', 'joint7'
        ]
        joint_msg.position = [float(j) for j in joint_positions]

        self.publisher_.publish(joint_msg)
        
        self.get_logger().info(
            f"Joint values: {', '.join([f'{j:.3f}' for j in joint_positions])}"
        )
        
        T = robot.fkine(joint_positions)
        
        xyz_result = T.t  # oppure T.t.tolist() se vuoi in forma di lista
        self.get_logger().info(f"Posizione effettiva (FK): {xyz_result}")
        return True

    else:
    
	    self.get_logger().info(f"Posizione non raggiungibile")
	    return False



class VRDataLogger(Node):
    def __init__(self):
        super().__init__('vr_data_logger')


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
        #o = pose_msg.pose.orientation

        # Orientamento fisso (quaternion unitario)
        class o:
            x = 0.0
            y = 0.0
            z = 0.0
            w = 1.0

        # extract grip
        grip = grip_msg.data

        # log to console without timestamp
        self.get_logger().info(
            f"pos=({p.x:.3f},{p.y:.3f},{p.z:.3f}) | "
            f"ori=({o.x:.3f},{o.y:.3f},{o.z:.3f},{o.w:.3f}) | grip={grip:.3f}"
        )

        x_robot, y_robot, z_robot = transform_input(p.x, p.y, p.z)

        # calculate inverse kinematic and publish joints
        cinematic(self,x_robot,y_robot,z_robot,o)
           

    def destroy_node(self):
        # close CSV
        try:
            #self.csv_file.close()
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
