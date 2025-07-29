import numpy as np
from visual_kinematics.RobotSerial import RobotSerial
from visual_kinematics.Frame import Frame
import csv


def cinematic(x,y,z):

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
        print("Soluzione trovata:")
        print("Valori dei giunti:", robot.axis_values)

        # 7. Esportazione dei dati in CSV
        joint_positions = [robot.axis_values.tolist()]
        with open('joint_positions.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7'])  # Intestazioni
            writer.writerows(joint_positions)
    else:
        print("La posizione desiderata non è raggiungibile.")
