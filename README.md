##  Nodi Disponibili e Modalità di Controllo

Il pacchetto `armando_controller` offre tre modalità principali per gestire il robot:

###  Controllo a Pose Predefinite
Permette di inviare il robot in posizioni articolari specifiche salvate nel file `config/poses.yaml`. Utilizza un **Action Client** per garantire movimenti fluidi e sincronizzati.

*   **Avvio:**
    ```bash
    ros2 run armando_controller armando_controller_node --ros-args -p pose:="pos0"

*(Sostituisci `pos0` con il nome della posa desiderata o `home` per resettare).*

### Automazione Completa 

Esegue una sequenza automatizzata temporizzata di "Pick and Place". Il robot si sposta tra diverse pose e attiva/disattiva i plugin Gazebo per afferrare e rilasciare i cubi.

*   **Avvio:**
    ```bash
    ros2 run armando_controller armando_pick_place_node

*   **Controllo Manuale Gripper:**
    È possibile forzare l'aggancio o lo sgancio dei cubi (ID 0-3) tramite i seguenti topic:
    ```bash
    # Aggancia cubo A (ID 0)
    ros2 topic pub --once /gripper/attach_a std_msgs/msg/Empty "{}"

    # Sgancia cubo A (ID 0)
    ros2 topic pub --once /gripper/detach_a std_msgs/msg/Empty "{}"

*Mappatura cubi: `a=ID 0`, `b=ID 1`, `c=ID 2`, `d=ID 3`.*

# Controllo Dinamico IK 

Questo nodo utilizza la Cinematica Inversa (IK) analitica per raggiungere i target (Marker ArUco) rilevati dalla telecamera. 


*   **Avvio:**
    ```bash
    ros2 launch armando_controller armando_ik.launch.py

*   **Cambio Target in Real-Time:**
    Cambia l'ID del cubo da inseguire senza riavviare il nodo:
    ```bash
    ros2 param set /armando_ik_node target_id 2

*(Usa l'ID desiderato o `-1` per far tornare il braccio in posizione Home).*


🔧 Comandi Utili e Debug
Inviare il robot in posizione Home (0,0,0,0)

Se vuoi resettare la posa del braccio manualmente tramite l'Action Server:
*  ```bash

    ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
        trajectory: {
            joint_names: ['j0', 'j1', 'j2', 'j3'],
            points: [{
                positions: [0.0, 0.0, 0.0, 0.0],
                velocities: [0.0, 0.0, 0.0, 0.0],
                time_from_start: {sec: 2, nanosec: 0}
            }]
        }
    }"
In alternativa, puoi usare il nodo controller con il parametro preimpostato:
*  ```bash
    ros2 run armando_controller armando_controller_node --ros-args -p pose:="home"