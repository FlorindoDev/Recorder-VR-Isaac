import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
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
        y_robot = y_in         # Isaac X → robot -Y
        z_robot = z_in          # Isaac Y → robot Z
        return x_robot, y_robot, z_robot

def cinematic2(self, x, y, z, o):
    dh_params = np.array([
        [0.34, 0., -np.pi / 2, 0.],
        [0., 0.,  np.pi / 2, 0.],
        [0.4, 0., -np.pi / 2, 0.],
        [0., 0.,  np.pi / 2, 0.],
        [0.4, 0., -np.pi / 2, 0.],
        [0., 0.,  np.pi / 2, 0.],
        [0.126, 0., 0., 0.]
    ])

    robot = RobotSerial(dh_params)
    xyz = np.array([[x], [y], [z]])
    r = R.from_quat([o.x, o.y, o.z, o.w])
    abc = r.as_euler('xyz')
    end = Frame.from_euler_3(abc, xyz)

    robot.inverse(end)

    if robot.is_reachable_inverse:
        joint_positions = robot.axis_values.tolist()

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'joint6', 'joint7'
        ]
        joint_msg.position = joint_positions

        self.publisher_.publish(joint_msg)

        self.get_logger().info(
            f"Joint values: {', '.join([f'{j:.3f}' for j in joint_positions])}"
        )

        T = robot.forward(robot.axis_values)
        xyz_result = T.t_3_1.reshape([3, ])
        self.get_logger().info(
                    f"Posizione effettiva (FK): {xyz_result}"
        )
     

        
        return True
    else:
        self.get_logger().warn("La posizione desiderata non è raggiungibile.")
        return False


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
        

    else:
    
	    self.get_logger().info(
		            f"Posizione non raggiungibile"
		)





class ManualJointCommander(Node):
    def __init__(self):
        super().__init__('manual_joint_commander')
        self.publisher_ = self.create_publisher(JointState, '/joint_command', 10)
        self.get_logger().info("Nodo pronto. Inserisci le coordinate x y z nel terminale.")

        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz loop

    def timer_callback(self):
        try:
            user_input = input("Inserisci x y z separati da spazio (CTRL+C per uscire): ")
            x_str, y_str, z_str = user_input.strip().split()
            x_input = float(x_str)
            y_input = float(y_str)
            z_input = float(z_str)

            x_robot, y_robot, z_robot = transform_input(x_input, y_input, z_input)
           
            

            # Orientamento fisso (quaternion unitario)
            class Orientation:
                x = 0.0
                y = 0.0
                z = 0.0
                w = 1.0

            cinematic(self, x_robot, y_robot, z_robot, Orientation)

        except ValueError:
            self.get_logger().error("Input non valido. Inserisci 3 numeri separati da spazi.")
        except Exception as e:
            self.get_logger().error(f"Errore: {e}")




def main(args=None):
    rclpy.init(args=args)
    node = ManualJointCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrotto da tastiera.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
