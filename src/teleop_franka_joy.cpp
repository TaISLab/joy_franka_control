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

#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "teleop_franka_joy/teleop_franka_joy.h"

#include <map>
#include <string>

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
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy); // Función para manejar los mensajes del joystick
  void sendCmdPoseStampedMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map); // Función para enviar comandos de PoseStamped

  ros::Subscriber joy_sub; // Subscriptor para el tema del joystick
  ros::Publisher position_pub; // Publicador para enviar comandos de PoseStamped

  int enable_button; // Vble que activa el control
  int enable_turbo_button; //Vble que activa la velocidad turbo
 
  // Creación de un map por cada joystick:
  std::map<std::string, int> JL_map; // Mapa que asigna el nombre de JL_map a un eje determinado de mando
  std::map< std::string, std::map<std::string, double> > scale_JL_map; // Mapa que asocia el nombre de un eje con una escala asociada al movimiento

  bool sent_disable_msg; // Bandera para indicar si se ha enviado un mensaje de desactivación
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

  pimpl_->position_pub = nh->advertise<geometry_msgs::Twist>("cmd_posestamped", 1, true); // Se crea el publicador ROS que publicará mensajes de tipo Twist en el topic cmd_posestamped
  
  pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopFrankaJoy::Impl::joyCallback, pimpl_); 
  // Se crea un subscriptor ROS que se subscribirá al topic joy y publica mensajes de tipo sensor_msgs::Joy. 
  // Cuando se recive un mensaje llama a la función callback.

  nh_param->param<int>("enable_button", pimpl_->enable_button, 0); // Se obtiene el parámetro del enable_button del servidor de parámetros ROS, por defecto es 0.
  nh_param->param<int>("enable_turbo_button", pimpl_->enable_turbo_button, -1);

  if (nh_param->getParam("JL_x", pimpl_->JL_map))
  {
    // Obtiene los parámetros axis_lineal

    nh_param->getParam("scale_JL_x", pimpl_->scale_JL_map["normal"]);
    nh_param->getParam("scale_JL_x_turbo", pimpl_->scale_JL_map["turbo"]);
  }
  else
  {
    // Si no se especifican: se aplican estos por defecto
    nh_param->param<int>("JL_x", pimpl_->JL_map["x"], 1);
    nh_param->param<double>("scale_JL_x", pimpl_->scale_JL_map["normal"]["x"], 0.5);
    nh_param->param<double>("scale_JL_x_turbo", pimpl_->scale_JL_map["turbo"]["x"], 1.0);
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

// Envia los comandos de velocidad
void TeleopFrankaJoy::Impl::sendCmdPoseStampedMsg(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                         const std::string& which_map)
{
  /*
    Método que envia un mensaje de velocidad asado en los datos del joystick
  */


  // Initializes with zeros by default.
  geometry_msgs::Point position_msg;

  position_msg.JL.x = getVal(joy_msg, JL_map, scale_JL_map[which_map], "x"); // Se obtiene el valor del eje x del joystick, escalandolo y asignandolo a la componente x del mensaje
  position_msg.JL.y = getVal(joy_msg, JL_map, scale_JL_map[which_map], "y");
  position_msg.JL.z = getVal(joy_msg, JL_map, scale_JL_map[which_map], "z");

  position_pub.publish(position_msg); // Se publica el mensaje de velocidad en el topic cmd_posestamped
  sent_disable_msg = false;
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
      // Initializes with zeros by default.
      geometry_msgs::Point position_msg;
      position_pub.publish(position_msg);
      sent_disable_msg = true;
    }
  }

}

}  // namespace teleop_franka_joy

// No hay main porque se asume que será proporcionado por un nodo ROS separado que instanciará y ejecutará este nodo
