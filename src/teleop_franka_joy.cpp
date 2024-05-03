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
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy); // Función encargada de manejar los mensajes del joystick
  void sendCmdPoseStampedMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map); // Función encargada de calcular los valores de PoseStamped
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  ros::Subscriber joy_sub; // Subscriptor a joystick
  ros::Subscriber equilibrium_pose_sub;
  ros::Publisher cmd_PoseStamped_pub; // Publicador de PoseStamped

  int enable_button; // Vble que activa el control
  int enable_turbo_button; //Vble que activa la velocidad turbo
  bool sent_disable_msg; // Bandera para indicar si se ha enviado un mensaje de desactivación
  bool received_equilibrium_pose;
  bool entra_una_vez;
  geometry_msgs::PoseStamped initial_pose;
 
  // Creación de un map por cada joystick:
  std::map<std::string, int> JL_map; // Mapa que asigna el nombre de JL_map a un eje determinado de mando
  std::map< std::string, std::map<std::string, double> > scale_JL_map; // Mapa que asocia el nombre de un eje con una escala asociada al movimiento

  
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
  pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopFrankaJoy::Impl::joyCallback, pimpl_); // Cuando se recive un mensaje llama a la función callback.
  
  pimpl_->equilibrium_pose_sub= nh->subscribe<geometry_msgs::PoseStamped>( 
      "/cartesian_impedance_example_controller/equilibrium_pose", 1, &TeleopFrankaJoy::Impl::equilibriumPoseCallback, pimpl_);

  nh_param->param<int>("enable_button", pimpl_->enable_button, 0); // Se obtiene el parámetro del enable_button del servidor de parámetros ROS, por defecto es 0.
  nh_param->param<int>("enable_turbo_button", pimpl_->enable_turbo_button, -1);

  if (nh_param->getParam("JL", pimpl_->JL_map))
  {
    // Obtiene los parámetros
    nh_param->getParam("scale_JL", pimpl_->scale_JL_map["normal"]);
    nh_param->getParam("scale_JL_turbo", pimpl_->scale_JL_map["turbo"]);
  }
  else
  {
    // Si no se especifican: se aplican estos por defecto
    nh_param->param<int>("JL", pimpl_->JL_map["x"], 1);
    nh_param->param<double>("scale_JL", pimpl_->scale_JL_map["normal"]["x"], 0.5);
    nh_param->param<double>("scale_JL_turbo", pimpl_->scale_JL_map["turbo"]["x"], 1.0);
  }
  ROS_INFO_NAMED("TeleopFrankaJoy", "Teleop enable button %i.", pimpl_->enable_button); // Imprime por pantalla
  ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopFrankaJoy", // Imprime por pantalla si la condición es verdadera
      "Turbo on button %i.", pimpl_->enable_turbo_button);

  // Bucle for que recorre el mapa JL_map. Cada elemento en ese mapa es un par clave-valor
  for (std::map<std::string, int>::iterator it = pimpl_->JL_map.begin();
      it != pimpl_->JL_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopFrankaJoy", "JL axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_JL_map["normal"][it->first]);

    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopFrankaJoy",
        "Turbo for JL axis %s is scale %f.", it->first.c_str(), pimpl_->scale_JL_map["turbo"][it->first]);
  }

  pimpl_->sent_disable_msg = false; // Establece el valor de la vble sent_disable_msg en false
}

// Obtiene valores específicos del mensaje del joystick
double getVal(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_map,
              const std::map<std::string, double>& scale_map, const std::string& fieldname)
{

  /*
  Método que obtiene valores especificos del mensaje del joystick:
  Argumentos:
    - joy_msg: puntero cte al mensaje del joystick
    - axis_map: mapa que asocia nombre con indices de ejes en el joy_stick
    - scale_map: mapa que asocia nombres de campos con escalas para esos campos
    - fieldname: Nombre del campo que se quiere obtener

  */

  if (axis_map.find(fieldname) == axis_map.end() ||
      scale_map.find(fieldname) == scale_map.end() ||
      joy_msg->axes.size() <= axis_map.at(fieldname))
  {
    // Condicional que verifica si fieldname existe en axis_map y scale_map, 
    // y si el tamaño del vector de ejes en joy_msg es mayor al indicado en axis_map devuelve 0
    return 0.0;
  }

  // Retorna el valor del eje especificado por fieldname en joy_msg, escalado por el valor asociado con fieldname en scale_map
  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

// Agrega la suscripción al topic equilibrium_pose
void TeleopFrankaJoy::Impl::equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  received_equilibrium_pose=true;
  initial_pose = *msg;

  // Imprime la posición y orientación recibida
  ROS_INFO("Equilibrium Pose- Position (x, y, z): (%.2f, %.2f, %.2f), Orientation (x, y, z, w): (%.2f, %.2f, %.2f, %.2f)",
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
            msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  
}

// Envia los comandos de PoseStamped
void TeleopFrankaJoy::Impl::sendCmdPoseStampedMsg(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                         const std::string& which_map)
{
  geometry_msgs::PoseStamped accumulated_pose;

  if (entra_una_vez == false && received_equilibrium_pose == true){
    entra_una_vez==true;
    ROS_INFO("Entra al mutex");
    accumulated_pose = initial_pose;
  }

  if (entra_una_vez == true) {

  geometry_msgs::Point Position_msg;
  // static geometry_msgs::PoseStamped accumulated_pose;

  Position_msg.x = getVal(joy_msg, JL_map, scale_JL_map[which_map], "x");
  Position_msg.y = getVal(joy_msg, JL_map, scale_JL_map[which_map], "y");

  accumulated_pose.pose.position.x += Position_msg.x;
  accumulated_pose.pose.position.y += Position_msg.y;

  ROS_INFO("Acumulation Pose- Position (x, y, z): (%.2f, %.2f, %.2f), Orientation (x, y, z, w): (%.2f, %.2f, %.2f, %.2f)",
            accumulated_pose.pose.position.x, accumulated_pose.pose.position.y, accumulated_pose.pose.position.z,
            accumulated_pose.pose.orientation.x, accumulated_pose.pose.orientation.y, accumulated_pose.pose.orientation.z, accumulated_pose.pose.orientation.w);


  cmd_PoseStamped_pub.publish(accumulated_pose);
  sent_disable_msg = false;

  }


}

// Esta función se llama cada vez que se recibe un mensjae del joystick. Decide que tipo de comando 
// de velocidad mandar al robot (normal o turbo) o si detener el movimiento del robot
void TeleopFrankaJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  if (enable_turbo_button >= 0 &&
      joy_msg->buttons.size() > enable_turbo_button &&
      joy_msg->buttons[enable_turbo_button])
  {
    sendCmdPoseStampedMsg(joy_msg, "turbo");
  }
  else if (joy_msg->buttons.size() > enable_button &&
           joy_msg->buttons[enable_button])
  {
    sendCmdPoseStampedMsg(joy_msg, "normal");
  }
  else
  {
    // When enable button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_disable_msg)
    {
      cmd_PoseStamped_pub.publish(initial_pose);
      sent_disable_msg = true;
    }
}
}

} // namespace teleop_franka_joy