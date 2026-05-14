JetMax ROS 2 Humble Workspace
Repositorio de prácticas del brazo robótico JetMax en ROS 2 Humble con Gazebo Classic, incluyendo:
    • movimiento básico del brazo,
    • exploración cartesiana,
    • pick and place con cubo,
    • visión artificial con cámara USB,
    • generación de escena en Gazebo,
    • y ordenamiento automático por color.

1. Requisitos
Este proyecto fue desarrollado sobre:
    • Ubuntu 22.04
    • ROS 2 Humble
    • Gazebo Classic 11
    • Python 3
    • OpenCV
    • cámara USB compatible con usb_cam

2. Estructura general
El workspace principal es:
~/tesis_ws
Los paquetes principales dentro de src son:
    • jetmax_description
    • jetmax_controller
    • jetmax_examples
    • jetmax_vision
    • IFRA_LinkAttacher

3. Instalación de dependencias
3.1 Dependencias base de ROS 2 y Gazebo
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-tf2-tools \
  ros-humble-rqt-graph
3.2 Dependencias para visión artificial
sudo apt install -y \
  python3-opencv \
  ros-humble-cv-bridge \
  ros-humble-usb-cam
3.3 Utilidades recomendadas
sudo apt install -y tree v4l-utils

4. Clonar el repositorio
Dentro del workspace:
mkdir -p ~/tesis_ws/src
cd ~/tesis_ws/src
git clone <git clone https://github.com/FrankNSQ/tesis_ros2_jetmax.git>

5. Compilación del workspace
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
Si solo se quiere compilar un paquete concreto:
colcon build --packages-select jetmax_vision
source install/setup.bash

6. Verificación de la cámara USB
Listar dispositivos disponibles:
ls /dev/video*
v4l2-ctl --list-devices
Probar la cámara:
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2
Cambiar /dev/video2 por el dispositivo correcto según el equipo.
Comprobar que hay imagen:
ros2 topic list | grep image
ros2 topic hz /image_raw
ros2 topic echo /image_raw --once

7. Ejemplo 1: saludo
Objetivo
Validar el movimiento básico del brazo en simulación.
Ejecución
Terminal 1
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch jetmax_description sim.launch.py
Terminal 2
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jetmax_controller arm_bridge_sim
Terminal 3
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jetmax_examples saludo

8. Ejemplo 2: exploración
Objetivo
Recorrer varios puntos cartesianos para validar trayectorias simples.
Ejecución
Terminal 1
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch jetmax_description sim.launch.py
Terminal 2
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jetmax_controller arm_bridge_sim
Terminal 3
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jetmax_examples exploracion

9. Ejemplo 3: pick and place con cubo
Objetivo
Recoger un cubo y colocarlo en otra posición mediante attachment en Gazebo.
Requisito
Deben existir los servicios:
ros2 service list | grep -i attach
Deben aparecer:
/ATTACHLINK
/DETACHLINK
Ejecución
Terminal 1
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch jetmax_description sim.launch.py
Terminal 2
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jetmax_controller arm_bridge_sim
Terminal 3
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch jetmax_description spawn_red_box.launch.py
Terminal 4
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jetmax_controller suction_attach_sim
Terminal 5
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jetmax_examples pick_and_place

10. Ejemplo 4: visión artificial y ordenamiento por color
Objetivo
Detectar cubos reales con una cámara USB, transferir la escena a Gazebo y ordenar los cubos con el robot simulado.
Nodos principales del paquete jetmax_vision
    • color_detector
    • calibrate_points
    • cube_state_publisher
    • gazebo_cube_spawner
    • gazebo_state_keeper
    • gazebo_sort_executor

10.1 Calibración de la hoja
Antes del ejemplo de visión, calibrar las cuatro esquinas de la hoja.
Terminal 1
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2
Terminal 2
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jetmax_vision calibrate_points
Hacer clic en este orden:
    1. esquina superior izquierda
    2. esquina superior derecha
    3. esquina inferior derecha
    4. esquina inferior izquierda
Luego copiar esos puntos dentro de color_detector.py.

10.2 Ejecución del sistema de visión
Terminal 1: Gazebo
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch jetmax_description sim.launch.py
Terminal 2: cámara
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2
Terminal 3: detector de color
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jetmax_vision color_detector
Terminal 4: estado estable de la escena
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jetmax_vision cube_state_publisher
Terminal 5: spawner en Gazebo
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jetmax_vision gazebo_cube_spawner
Terminal 6: memoria persistente del estado
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jetmax_vision gazebo_state_keeper
Terminal 7: puente cartesiano
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jetmax_controller arm_bridge_sim
Terminal 8: ejecutor de ordenamiento
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run jetmax_vision gazebo_sort_executor

11. Tópicos principales
Movimiento
    • /arm/set_position
    • /arm_controller/joint_trajectory
Visión
    • /image_raw
    • /vision/detected_color
    • /vision/detected_cube
    • /vision/cube_state
Estado de la escena en Gazebo
    • /gazebo_cube_state_raw
    • /gazebo_cube_state

12. Servicios principales
/ATTACHLINK
/DETACHLINK

13. Herramientas útiles de inspección
Ver nodos
ros2 node list
Ver tópicos
ros2 topic list
Ver información de un tópico
ros2 topic info /vision/cube_state
Grafo de nodos
rqt_graph
Ver estructura del workspace
tree -L 4 ~/tesis_ws/src

14. Recomendaciones
    • recompilar el workspace cada vez que se modifique un nodo Python,
    • mantener fija la cámara en el ejemplo de visión,
    • recalibrar la hoja si cambia el encuadre o la posición de la cámara,
    • reiniciar Gazebo cuando la escena quede en un estado inconsistente,
    • y documentar cada ejemplo con capturas de Gazebo, rqt_graph y tópicos principales.

15. Qué demuestra este repositorio
Este workspace permite desarrollar prácticas progresivas:
    1. saludo,
    2. exploración,
    3. pick and place con cubo,
    4. visión artificial con ordenamiento automático.
El último ejemplo integra percepción, simulación y manipulación dentro de un único flujo de trabajo.

16. Notas finales
El sistema de visión usa una hoja calibrada y una homografía planar para transformar coordenadas de imagen en coordenadas útiles sobre la superficie de trabajo. Posteriormente, dicha escena se traslada a Gazebo para que el robot simulado pueda manipular los cubos.
Este repositorio está orientado a fines didácticos, de simulación y de investigación aplicada en robótica y visión artificial.
