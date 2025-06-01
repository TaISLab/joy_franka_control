# joy_franka_control

Este paquete de ROS se ha desarrollado para teleoperar el controlador de impedancia cartesiana del manipulador Franka Emika Panda mediante un mando linux. La creación de este paquete se enmarca dentro del proyecto RAFI.

## Funcionamiento
Este paquete contiene el nodo `joy_franka_node`. Este nodo se subscribe al topic `/Joy` para obtener el estado del mando en forma de mensaje `sensor_msg/joy`. Usa esa información para mandar comandos de pose al controlador del manipulador. Para ello, se subcribe en el topic `/cartesian_impedance_example_controller/equilibrium_pose` y publica un mensaje de `Geometry_msg/PoseStamped`.

Además, se subcribe al topic `/franka_state_controller/franka_states` para obtener el `equilibrium_pose` inicial al momento de iniciar la teleoperación.

## Instalación

### Paquetes necesarios
Para su uso, se debe tener instalados los siguientes paquetes. Se adjunta el enlace que contiene información sobre su instalación.
- `joy`. Driver para el mando. Disponible en: http://wiki.ros.org/joy.
- `franka_ros_rafi'. Se trata de una versión modificada del metapaquete `franka_ros`. <!-- añadir enlace al metapaquete modificado. lo que se ha modificado  -->

<!-- Se debe tener instalado y configurado el metapaquete `franka_ros` así como la librería `libfranka`. Disponible en: https://frankaemika.github.io/docs/installation_linux.html. -->

### Instalación de este paquete

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

## Cómo comprobar si tu mando admite vibración (force‐feedback)

Estos pasos te muestran cómo configurar `joy_node` para abrir el dispositivo de eventos asociado a tu mando y, a continuación, cómo enviar un comando de vibración para comprobar si funciona.

### 1. Identificar el “event” de tu mando

1. Conecta tu mando al puerto USB.
2. En una terminal, ejecuta:
   ```bash
   sudo udevadm monitor --kernel
   ```

3. Desconecta y vuelve a conectar el mando. Observa la salida y busca una línea similar a:
```
KERNEL[1234.567890] add      /devices/.../input/inputNN/eventXX (input)
```

Anota el nombre completo, por ejemplo /dev/input/event21.

4. Modifica el launch

```
<launch>
  <!-- Ruta al dispositivo de lectura de ejes/botones (por lo general js0) -->
  <arg name="joy_dev"   default="/dev/input/js0"/>
  <!-- Ruta al dispositivo de eventos que debería soportar fuerza (reemplazar eventNN) -->
  <arg name="joy_ff"    default="/dev/input/eventNN"/>
  <arg name="joy_topic" default="/joy"/>

  <node pkg="joy" type="joy_node" name="joy_node" output="screen">
    <param name="dev"     value="$(arg joy_dev)"/>
    <param name="deadzone" value="0.3"/>
    <param name="dev_ff"  value="$(arg joy_ff)"/>
    <param name="autorepeat_rate" value="20"/>
    <remap from="joy" to="$(arg joy_topic)"/>
  </node>
</launch>
```

5. Mandar una vibración de prueba

```bash
rostopic pub --once /joy/set_feedback sensor_msgs/JoyFeedbackArray "array:
  - type: 1
    id: 0
    intensity: 1.0"
```
Parámetros: 
- type: 1 → TYPE_RUMBLE (vibración).
- id: 0 → primer motor de vibración (muchos mandos solo tienen un motor y lo usan con identificación 0).
- intensity: 1.0 → máxima intensidad (entre 0.0 y 1.0).

Si tu mando sí soporta vibración, lo sentirás en ese momento. Si no notas ninguna vibración, es porque el driver de Linux no expone motor de force‐feedback para ese dispositivo.