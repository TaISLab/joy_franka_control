# joy_franka_control

Este paquete de ROS se ha desarrollado para teleoperar el controlador de impedancia cartesiana del manipulador Franka Emika Panda mediante un mando linux. La creación de este paquete se enmarca dentro del proyecto RAFI.

## Funcionamiento
Este paquete contiene el nodo `joy_franka_node`. Este nodo se subscribe al topic `/Joy` para obtener el estado del mando en forma de mensaje `sensor_msg/joy`. Usa esa información para mandar comandos de pose al controlador del manipulador. Para ello, se subcribe en el topic `/cartesian_impedance_example_controller/equilibrium_pose` y publica un mensaje de `Geometry_msg/PoseStamped`.

Además, se subcribe al topic `/franka_state_controller/franka_states` para obtener el `equilibrium_pose` inicial al momento de iniciar la teleoperación.

## Paquetes necesarios

Se debe tener instalado el paquete `joy` encargado de actuar como driver para el mando. Disponible en: http://wiki.ros.org/joy.

Se debe tener instalado y configurado el metapaquete `franka_ros` así como la librería `libfranka`. Disponible en: https://frankaemika.github.io/docs/installation_linux.html.

## Instalación

```bash
git clone https://github.com/rodri-castro/joy_franka_control.git
catkin_make
```
## Uso

```bash
roslaunch joy_franka_control teleop_franka.launch
```
Este comando lanza el nodo del driver del mando junto con el nodo `joy_franka_node`. Este archivo de lanzamiento no lanza el controlador de impedancia cartesiana del manipulador.

En el caso de querer lanzar exclusivamente el nodo de este paquete:
```bash
roslaunch joy_franka_control teleop_franka_without_joy.launch
```

