import numpy as np
from visual_kinematics.RobotSerial import RobotSerial
from visual_kinematics.Frame import Frame

def cinematic(x,y,z, get_logger):

    # 1. Definizione dei parametri DH per il KUKA LBR iiwa 7 R800
    dh_params = np.array([
        [0.34, 0., -np.pi / 2, 0.],
        [0., 0.,  np.pi / 2, 0.],
        [0.4, 0., -np.pi / 2, 0.],
        [0., 0.,  np.pi / 2, 0.],
        [0.4, 0., -np.pi / 2, 0.],
        [0., 0.,  np.pi / 2, 0.],
        [0.126, 0., 0., 0.]
    ])

    # 2. Creazione del robot
    robot = RobotSerial(dh_params)

    # 3. Definizione della posa desiderata dell'end-effector
    xyz = np.array([[x], [y], [z]])  # Posizione in metri
    abc = np.array([0.5 * np.pi, 0., np.pi])      # Orientamento in radianti (ZYX)

    # 4. Creazione del frame desiderato
    end = Frame.from_euler_3(abc, xyz)

    # 5. Calcolo della cinematica inversa
    robot.inverse(end)

    # 6. Verifica se la soluzione è raggiungibile
    if robot.is_reachable_inverse:
        get_logger.info(f"{robot.axis_values}")
        print("Soluzione trovata:")
        print("Valori dei giunti:", robot.axis_values)

    else:
        print("La posizione desiderata non è raggiungibile.")
