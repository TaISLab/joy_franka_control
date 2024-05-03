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

  geometry_msgs::PoseStamped initial_pose;
  int enable_button; // Variable que activa el control
  int enable_turbo_button; //Variable que activa la velocidad turbo
  bool sent_disable_msg; // Variable para indicar si se ha enviado un mensaje de desactivación
  bool received_equilibrium_pose;
  int orientation_button;
  int home_button;

  // Exclusión mutua
  std::mutex mtx;
 
  // Creación de un map por cada joystick:
  std::map<std::string, int> JL_map; // Mapa que asigna el nombre de JL_map a un eje determinado de mando
  std::map< std::string, std::map<std::string, double> > scale_JL_map; // Mapa que asocia el nombre de un eje con una escala asociada al movimiento

  // Map para control de altura
  std::map< std::string, double> scale_Z_map;
  int change_z_button;
  bool mov_z_arriba;

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

  nh_param->param<int>("enable_button", pimpl_->enable_button, 0); // Se obtiene el parámetro del enable_button del servidor de parámetros ROS, por defecto es 0.
  nh_param->param<int>("enable_turbo_button", pimpl_->enable_turbo_button, -1);
  nh_param->param<int>("orientation_button", pimpl_->orientation_button, -1); // Antes 8
  nh_param->param<int>("home_button", pimpl_->home_button, -1);
  nh_param->param<int>("change_z_button", pimpl_->change_z_button, -1);

  if (nh_param->getParam("JL", pimpl_->JL_map)) // Obtiene los parámetros
  {
    nh_param->getParam("scale_JL", pimpl_->scale_JL_map["normal"]);
    nh_param->getParam("scale_JL_turbo", pimpl_->scale_JL_map["turbo"]);
  }
  // else // Si no se especifican: se aplican estos por defecto
  // {
  //   nh_param->param<int>("JL", pimpl_->JL_map["x"], 1);
  //   nh_param->param<double>("scale_JL", pimpl_->scale_JL_map["normal"]["x"], 0.5);
  //   nh_param->param<double>("scale_JL_turbo", pimpl_->scale_JL_map["turbo"]["x"], 1.0);
  // }

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
  pimpl_->received_equilibrium_pose = false;

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

// Función del subscriptor: obtiene la equilibrium pose
void TeleopFrankaJoy::Impl::equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  if (received_equilibrium_pose == false){

    ROS_INFO("Me subscribo a equilibriumPose");
    received_equilibrium_pose=true;

    mtx.lock();
    initial_pose = *msg;

    // Imprime la posición y orientación recibida
    ROS_INFO("Subscripcion: Equilibrium Pose- Position (x, y, z): (%.2f, %.2f, %.2f), Orientation (x, y, z, w): (%.2f, %.2f, %.2f, %.2f)",
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
              msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    mtx.unlock();
  }
}

double applyLimits(double value, double min_limit, double max_limit){
  return std::min(std::max(value, min_limit), max_limit);
}


// Función del publicador: envia los comandos de PoseStamped
void TeleopFrankaJoy::Impl::sendCmdPoseStampedMsg(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                         const std::string& which_map)
{
  geometry_msgs::PoseStamped initial_pose_local = initial_pose;
  geometry_msgs::Point Position_msg;

  Position_msg.x = initial_pose_local.pose.position.x + getVal(joy_msg, JL_map, scale_JL_map[which_map], "x");
  Position_msg.y = initial_pose_local.pose.position.y + getVal(joy_msg, JL_map, scale_JL_map[which_map], "y");
  Position_msg.z = initial_pose_local.pose.position.z + getVal(joy_msg, JL_map, scale_JL_map[which_map], "z");

  initial_pose_local.pose.position.x = applyLimits(Position_msg.x, -15, 15);
  initial_pose_local.pose.position.y = applyLimits(Position_msg.y, -15, 15);
  initial_pose_local.pose.position.z = applyLimits(Position_msg.z, -5, 10);

  ROS_INFO("Desired Pose- Position (x, y, z): (%.2f, %.2f, %.2f), Orientation (x, y, z, w): (%.2f, %.2f, %.2f, %.2f)",
            initial_pose_local.pose.position.x, initial_pose_local.pose.position.y, initial_pose_local.pose.position.z,
            initial_pose_local.pose.orientation.x, initial_pose_local.pose.orientation.y, initial_pose_local.pose.orientation.z, initial_pose_local.pose.orientation.w);
  
  mtx.lock();
  initial_pose = initial_pose_local;
  sent_disable_msg = false;
  mtx.unlock();
  cmd_PoseStamped_pub.publish(initial_pose);

}


// Esta función se llama cada vez que se recibe un mensjae del joystick. Decide que tipo de comando 
// de velocidad mandar al robot (normal o turbo) o si detener el movimiento del robot
void TeleopFrankaJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  if (joy_msg->buttons.size() > orientation_button &&
      joy_msg->buttons[orientation_button]){
    ROS_INFO("Orientation button press: %d", orientation_button);
    sent_disable_msg = true;

    mtx.lock();
    initial_pose.pose.orientation.x = 0.0;
    initial_pose.pose.orientation.y = 0.0;
    initial_pose.pose.orientation.z = 0.0;
    initial_pose.pose.orientation.w = 1.0;
    mtx.unlock();

    cmd_PoseStamped_pub.publish(initial_pose);

  } else if (joy_msg->buttons.size() > home_button &&
    joy_msg->buttons[home_button]){

    ROS_INFO("Home button press: %d", home_button);

    mtx.lock();
    initial_pose.pose.position.x = 0.0;
    initial_pose.pose.position.y = 0.0;
    initial_pose.pose.position.z = 0.0;

    initial_pose.pose.orientation.x = 0.0;
    initial_pose.pose.orientation.y = 0.0;
    initial_pose.pose.orientation.z = 0.0;
    initial_pose.pose.orientation.w = 1.0;
    mtx.unlock();

    cmd_PoseStamped_pub.publish(initial_pose);

  } else if (enable_turbo_button >= 0 &&
      joy_msg->buttons.size() > enable_turbo_button &&
      joy_msg->buttons[enable_turbo_button])
  {
    ROS_INFO("enable_turbo button press: %d", enable_turbo_button);
    sendCmdPoseStampedMsg(joy_msg, "turbo");
  }
  else if (joy_msg->buttons.size() > enable_button &&
           joy_msg->buttons[enable_button])
  {
    sendCmdPoseStampedMsg(joy_msg, "normal");
  }
  else if (!sent_disable_msg) // When enable button is released
    {
      ROS_INFO("Repose Pose- Position (x, y, z): (%.2f, %.2f, %.2f), Orientation (x, y, z, w): (%.2f, %.2f, %.2f, %.2f)",
            initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z,
            initial_pose.pose.orientation.x, initial_pose.pose.orientation.y, initial_pose.pose.orientation.z, initial_pose.pose.orientation.w);
      received_equilibrium_pose=false;
      cmd_PoseStamped_pub.publish(initial_pose);
      sent_disable_msg = true;
    }
}
} // namespace teleop_franka_joy