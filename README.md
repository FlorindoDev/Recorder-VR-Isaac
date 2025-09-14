# Recorder-VR-Isaac

## 🇬🇧 English Version

### Overview

Recorder-VR-Isaac is a research project designed to link **Virtual
Reality (VR)** systems with robotics simulation through **NVIDIA Isaac
Sim** and **ROS2**. The main idea is to let a user control a robotic arm
in simulation using VR controllers, while also recording these movements
for future use.

### Features

-   **VR-based Robot Control**: Maps head and hand movements (Oculus
    Quest 2) into the simulated KUKA LBR iiwa R800.\
-   **Inverse Kinematics**: Transforms VR controller poses into joint
    commands.\
-   **ROS2 Integration**: Nodes for communication between VR,
    simulation, and IK solver.\
-   **Data Recording**: Outputs trajectories in `.csv` and `.npz`
    formats for ML or replay.\
-   **Simulation Environment**: Safe testing before real robot
    deployment.

### Technologies

-   NVIDIA Isaac Sim\
-   ROS2\
-   OpenVR & SteamVR\
-   ALVR (wireless streaming for Oculus Quest)

### Project Structure

-   `ros2_nodes/` → ROS2 nodes (data acquisition, IK, control).\
-   `data/` → Recorded datasets.\
-   `scripts/` → Utilities for recording and replay.

### Results

-   Accurate tracking (within \~0.5--1 cm).\
-   Intuitive and natural robot control via VR.\
-   Generation of reusable datasets for research.

------------------------------------------------------------------------

## 🇮🇹 Versione Italiana

### Panoramica

Recorder-VR-Isaac è un progetto di ricerca che integra la **Realtà
Virtuale (VR)** con la simulazione robotica tramite **NVIDIA Isaac Sim**
e **ROS2**. L'obiettivo è consentire all'utente di controllare un
braccio robotico in simulazione con i controller VR e registrare i
movimenti per un utilizzo successivo.

### Funzionalità

-   **Controllo VR → Robot**: Tracciamento dei movimenti di testa e mani
    (Oculus Quest 2) sul robot simulato KUKA LBR iiwa R800.\
-   **Cinematica inversa**: Conversione delle pose VR in comandi
    articolari.\
-   **Integrazione ROS2**: Nodi per la comunicazione fra VR, simulazione
    e solver IK.\
-   **Registrazione dati**: Salvataggio traiettorie in `.csv` e `.npz`
    per ML o riproduzione.\
-   **Ambiente di simulazione**: Validazione sicura prima di testare sul
    robot reale.

### Tecnologie

-   NVIDIA Isaac Sim\
-   ROS2\
-   OpenVR & SteamVR\
-   ALVR (streaming wireless per Oculus Quest)

### Struttura del progetto

-   `ros2_nodes/` → Nodi ROS2 (acquisizione dati, IK, controllo).\
-   `data/` → Dataset registrati.\
-   `scripts/` → Script di utilità per registrazione e riproduzione.

### Risultati

-   Precisione di tracciamento \~0.5--1 cm.\
-   Controllo intuitivo e naturale del robot in VR.\
-   Creazione di dataset utili per ricerca e ML.
