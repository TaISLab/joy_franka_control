/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "ros/ros.h"
#include <map>
#include <string>

#include "teleop_franka_joy/teleop_franka_joy.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Joy.h"

// Definir un namespace evita conflictos de nombres con otras partes del código o blibliotecas externas
namespace teleop_franka_joy
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopFrankaJoy
 * directly into base nodes.
 */
struct TeleopFrankaJoy::Impl
{
  // Members functions
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy); // Función encargada de manejar los mensajes del joystick
  void sendCmdPositionMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_position_map); // Función encargada de calcular los valores de PoseStamped
  void sendCmdOrientationMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_orientation_map);
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void ModifyVelocity(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_map, float& scale); // Función encargada de subir y bajar la velocidad
  bool AlcanzadoDesiredEquilibriumPose();
  void pseudoestatico();

  // ROS subscribers and publisher
  ros::Subscriber joy_sub;
  ros::Subscriber equilibrium_pose_sub; 
  ros::Publisher cmd_PoseStamped_pub;

  geometry_msgs::PoseStamped equilibrium_pose;
  geometry_msgs::PoseStamped nuevo_equilibrium_pose;
  float Delta_t = 0.001; // Tiempo en segundo
  float reaction_t = 0.5; // Tiempo en segundo


  int enable_mov_position; // Variable que activa el control
  int enable_mov_orientation; //Variable que activa la velocidad orientation
  bool sent_disable_msg; // Variable para indicar si se ha enviado un mensaje de desactivación
  bool actualizar_equilibrium_pose;
  int orientation_button;
  int home_button;
  int increment_vel;
  int decrement_vel;

  float position_max_vel;
  float orientation_max_vel; // max_displacement_in_a_second

  // Exclusión mutua
  std::mutex mutex; // Quitar
 
  // Creación de un map de ejes por cada tipo de control:
  std::map<std::string, int> axis_position_map; // Control de posicion
  std::map<std::string, int> axis_orientation_map; // Control de orientación

};

/**
 * Constructs TeleopFrankaJoy.
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
// Constructor: Inicializa los parámetros del nodo ROS y los parámetros del joystick
TeleopFrankaJoy::TeleopFrankaJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
{
  pimpl_ = new Impl;
  pimpl_->cmd_PoseStamped_pub = nh->advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_example_controller/equilibrium_pose", 1, true); // Se crea el publicador ROS que publicará mensajes de tipo PoseStamped en el topic cmd_posestamped
  pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopFrankaJoy::Impl::joyCallback, pimpl_); // Cuando se recibe un mensaje llama a la función callback.
  pimpl_->equilibrium_pose_sub= nh->subscribe<geometry_msgs::PoseStamped>( 
      "/cartesian_impedance_example_controller/equilibrium_pose", 1, &TeleopFrankaJoy::Impl::equilibriumPoseCallback, pimpl_);

  // Mapear botones
  nh_param->param<int>("enable_mov_position", pimpl_->enable_mov_position, 0); // Se obtiene el parámetro del enable_mov_position del servidor de parámetros ROS, por defecto es 0.
  nh_param->param<int>("enable_mov_orientation", pimpl_->enable_mov_orientation, -1);
  nh_param->param<int>("orientation_button", pimpl_->orientation_button, -1); // Antes 8
  nh_param->param<int>("home_button", pimpl_->home_button, -1);

  nh_param->param<int>("increment_velocity", pimpl_->increment_vel, -1);
  nh_param->param<int>("decrement_velocity", pimpl_->decrement_vel, -1);

  // Asignación de mapas
  nh_param->getParam("axis_position_map", pimpl_->axis_position_map);
  nh_param->getParam("axis_orientation_map", pimpl_->axis_orientation_map);

  nh_param->getParam("position_max_displacement_in_a_second", pimpl_->position_max_vel);
  nh_param->getParam("orientation_max_displacement_in_a_second", pimpl_->orientation_max_vel);

  pimpl_->sent_disable_msg = false; // Establece el valor de la vble sent_disable_msg en false
  pimpl_->actualizar_equilibrium_pose = true;

}

// Obtiene valores específicos del mensaje del joystick
double getVal(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_map, const std::string& fieldname)
{

  /*
  Método que obtiene valores especificos del mensaje del joystick:
  Argumentos:
    - joy_msg: mensaje joy del cual se va a obtener la informacion
    - axis_map: mapa de ejes de control
    - fieldname: campo que se quiere obtener [x,y,z] o [x,y,z,w]
  */

  if (axis_map.find(fieldname) == axis_map.end() ||
      joy_msg->axes.size() <= axis_map.at(fieldname))
  {
    // Condicional que verifica si fieldname existe en axis_map y scale_map, 
    // y si el tamaño del vector de ejes en joy_msg es mayor al indicado en axis_map devuelve 0
    return 0.0;
  }

  // Retorna el valor del eje especificado por fieldname en joy_msg, escalado por el valor asociado con fieldname en scale_map
  return joy_msg->axes[axis_map.at(fieldname)];
}

// Función del subscriptor: obtiene la equilibrium pose
void TeleopFrankaJoy::Impl::equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    // ROS_INFO("Me subscribo a equilibriumPose");
    
    nuevo_equilibrium_pose = *msg;

    // Imprime la posición y orientación recibida
    // ROS_INFO("Subscripcion: Equilibrium Pose- Position (x, y, z): (%.2f, %.2f, %.2f), Orientation (x, y, z, w): (%.2f, %.2f, %.2f, %.2f)",
    //           msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
    //           msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
}

double applyLimits(double value, double min_limit, double max_limit){
  return std::min(std::max(value, min_limit), max_limit);
}

void TeleopFrankaJoy::Impl::ModifyVelocity(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                           const std::map<std::string, int>& axis_map,
                                           float& scale) {
    if (joy_msg->buttons[increment_vel]) {
        scale = scale*2; // Incremento de la escala
        ROS_INFO("Velocidad incrementada a %f", scale);
    } else if (joy_msg->buttons[decrement_vel]) {
        scale = scale/2; // Decremento de la escala
        ROS_INFO("Velocidad decrementada a %f", scale);
    }
    ros::Duration(reaction_t).sleep(); // Espera de Delta_t segundos
}


bool TeleopFrankaJoy::Impl::AlcanzadoDesiredEquilibriumPose(){

  // Calcular la distancia euclidiana entre la posición actual y la deseada del equilibrium_pose
    double distance = sqrt(pow(equilibrium_pose.pose.position.x - nuevo_equilibrium_pose.pose.position.x, 2) +
                           pow(equilibrium_pose.pose.position.y - nuevo_equilibrium_pose.pose.position.y, 2) +
                           pow(equilibrium_pose.pose.position.z - nuevo_equilibrium_pose.pose.position.z, 2));

    // Comparar la distancia con un umbral predefinido
    double distance_threshold = 0.001;  // Umbral de distancia predefinido (ajusta según sea necesario)
    if (distance < distance_threshold) {
        // Si la distancia es menor que el umbral, se considera que se ha alcanzado la posición deseada
        return true;
        ROS_INFO("Equilibrium Pose alcanzado");
    } else {
        // Si la distancia es mayor que el umbral, aún no se ha alcanzado la posición deseada
        return false;
    }
}

void TeleopFrankaJoy::Impl::sendCmdPositionMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_position_map)
  {
    
    geometry_msgs::Point increment_position;

    if (actualizar_equilibrium_pose == true){
    equilibrium_pose = nuevo_equilibrium_pose;
    ROS_INFO("Me subscribo a equilibriumPose por primera vez");
    actualizar_equilibrium_pose = false;
    }
    // Calculo incrementos
    increment_position.x = Delta_t * position_max_vel * getVal(joy_msg, axis_position_map, "x");
    increment_position.y = Delta_t * position_max_vel * getVal(joy_msg, axis_position_map, "y");
    increment_position.z = Delta_t * position_max_vel * getVal(joy_msg, axis_position_map, "z");

    equilibrium_pose.pose.position.x += increment_position.x;
    equilibrium_pose.pose.position.y += increment_position.y;
    equilibrium_pose.pose.position.z += increment_position.z;

    // Aplicamos límites a cada uno de los ejes
    equilibrium_pose.pose.position.x = applyLimits(equilibrium_pose.pose.position.x, -15, 15);
    equilibrium_pose.pose.position.y = applyLimits(equilibrium_pose.pose.position.y, -15, 15);
    equilibrium_pose.pose.position.z = applyLimits(equilibrium_pose.pose.position.z, -15, 15);

    cmd_PoseStamped_pub.publish(equilibrium_pose);

    ROS_INFO("Desired Pose- Position (x, y, z): (%.2f, %.2f, %.2f), Orientation (x, y, z, w): (%.2f, %.2f, %.2f, %.2f)",
            equilibrium_pose.pose.position.x, equilibrium_pose.pose.position.y, equilibrium_pose.pose.position.z,
            equilibrium_pose.pose.orientation.x, equilibrium_pose.pose.orientation.y, equilibrium_pose.pose.orientation.z, equilibrium_pose.pose.orientation.w);

    ros::Duration(Delta_t).sleep(); // Espera de Delta_t segundos
    ROS_INFO("Espera de Delta_t completada.");

    if (AlcanzadoDesiredEquilibriumPose()){
      equilibrium_pose = nuevo_equilibrium_pose;
      ROS_INFO("Alcanzado EquilibriumPose deseado");
      actualizar_equilibrium_pose = false;
    }

  }

  void TeleopFrankaJoy::Impl::sendCmdOrientationMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_orientation_map)
  {
    geometry_msgs::Quaternion increment_orientation;

    increment_orientation.x = Delta_t * orientation_max_vel * getVal(joy_msg, axis_orientation_map, "x");
    increment_orientation.y = Delta_t * orientation_max_vel * getVal(joy_msg, axis_orientation_map, "y");
    increment_orientation.z = Delta_t * orientation_max_vel * getVal(joy_msg, axis_orientation_map, "z");
    increment_orientation.w = Delta_t * orientation_max_vel * getVal(joy_msg, axis_orientation_map, "w");

    equilibrium_pose.pose.orientation.x += increment_orientation.x;
    equilibrium_pose.pose.orientation.y += increment_orientation.y;
    equilibrium_pose.pose.orientation.z += increment_orientation.z;
    equilibrium_pose.pose.orientation.w += increment_orientation.w;

    cmd_PoseStamped_pub.publish(equilibrium_pose);

    ROS_INFO("Desired Pose- Position (x, y, z): (%.2f, %.2f, %.2f), Orientation (x, y, z, w): (%.2f, %.2f, %.2f, %.2f)",
            equilibrium_pose.pose.position.x, equilibrium_pose.pose.position.y, equilibrium_pose.pose.position.z,
            equilibrium_pose.pose.orientation.x, equilibrium_pose.pose.orientation.y, equilibrium_pose.pose.orientation.z, equilibrium_pose.pose.orientation.w);

    ros::Duration(Delta_t).sleep(); // Espera de Delta_t segundos
    ROS_INFO("Espera de Delta_t completada.");
  }

void TeleopFrankaJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  if (joy_msg->buttons[enable_mov_position]) //Boton derecho
  {
    ROS_INFO("Boton LB pulsado");
    if (joy_msg->buttons[increment_vel] || joy_msg->buttons[decrement_vel]){
    // Variar velocidad considerando que estamos en enable_mov_position
    ModifyVelocity(joy_msg, axis_position_map, position_max_vel);
    } else if(joy_msg->buttons[home_button]){ // LLeva a pto de reposo

      equilibrium_pose.pose.position.x = 0;
      equilibrium_pose.pose.position.y = 0;
      equilibrium_pose.pose.position.z = 0.5;

      cmd_PoseStamped_pub.publish(equilibrium_pose);

    } else{
    sendCmdPositionMsg(joy_msg, axis_position_map);
    }
  }else if (joy_msg->buttons[enable_mov_orientation]) // Boton izquierdo
  {
    ROS_INFO("Boton RB pulsado");
    if (joy_msg->buttons[increment_vel] || joy_msg->buttons[decrement_vel]){
    // Variar velocidad considerando que estamos en enable_mov_orientation
    ModifyVelocity(joy_msg, axis_orientation_map, orientation_max_vel);
    } else if(joy_msg->buttons[home_button]){ // LLeva a pto de reposo angular
      
      equilibrium_pose.pose.orientation.x = 0;
      equilibrium_pose.pose.orientation.y = 0;
      equilibrium_pose.pose.orientation.z = 0;
      equilibrium_pose.pose.orientation.w = 0;

      cmd_PoseStamped_pub.publish(equilibrium_pose);

    }else{
    sendCmdOrientationMsg(joy_msg, axis_orientation_map);
    }  
  }else{ // Si no se toca nada
    
      if (AlcanzadoDesiredEquilibriumPose()){
      equilibrium_pose = nuevo_equilibrium_pose;
      ROS_INFO("Me subscribo a equilibriumPose");
      }

      cmd_PoseStamped_pub.publish(equilibrium_pose); // Se publica el equilibrium_pose cuando no se pulsa ninguna tecla
      //ros::Duration(reaction_t).sleep(); // Espera de Delta_t segundos

      ROS_INFO("Repose Pose publishing- Position (x, y, z): (%.2f, %.2f, %.2f), Orientation (x, y, z, w): (%.2f, %.2f, %.2f, %.2f)",
        equilibrium_pose.pose.position.x, equilibrium_pose.pose.position.y, equilibrium_pose.pose.position.z,
        equilibrium_pose.pose.orientation.x, equilibrium_pose.pose.orientation.y, equilibrium_pose.pose.orientation.z, equilibrium_pose.pose.orientation.w);
    // ros::Duration(Delta_t).sleep(); // Prueba: eliminar el temblor

  }
}

} // namespace teleop_franka_joy