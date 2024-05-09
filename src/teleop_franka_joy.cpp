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
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "teleop_franka_joy/teleop_franka_joy.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Joy.h"
#include "franka_msgs/FrankaState.h"


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
  void printEquilibriumPoseInfo(const geometry_msgs::PoseStamped& equilibrium_pose, const std::string& info_string);

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy); // Función encargada de manejar los mensajes del joystick
  void sendCmdPositionMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_position_map); // Función encargada de calcular los valores de PoseStamped
  void sendCmdOrientationMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_orientation_map);
  void ModifyVelocity(const sensor_msgs::Joy::ConstPtr& joy_msg, float& scale, float& max_vel); // Función encargada de modificar la velocidad
  void obtainEquilibriumPose(const franka_msgs::FrankaStateConstPtr& msg);

  // ROS subscribers and publisher
  ros::Subscriber joy_sub;
  ros::Subscriber franka_state_sub; // Obtiene la tf 0_T_EE
  ros::Publisher cmd_PoseStamped_pub;

  franka_msgs::FrankaState current_franka_state;
  geometry_msgs::PoseStamped equilibrium_pose;

  float Delta_t = 0.01; // Tiempo en segundo
  float reaction_t = 0.5; // Tiempo en segundo de reaccion del operador

  int enable_mov_position; // Variable que activa el control
  int enable_mov_orientation; //Variable que activa la velocidad orientation
  int orientation_button;
  int home_button;
  int increment_vel;
  int decrement_vel;

  float position_vel = 5;
  float orientation_vel = 20;
  float position_max_vel;
  float orientation_max_vel; // max_displacement_in_a_second
  float min_vel = 2;
 
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
  pimpl_->franka_state_sub = nh->subscribe<franka_msgs::FrankaState>("/franka_state_controller/franka_states", 10, &TeleopFrankaJoy::Impl::obtainEquilibriumPose, pimpl_);

  // Asignar botones
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

}

void TeleopFrankaJoy::Impl::printEquilibriumPoseInfo(const geometry_msgs::PoseStamped& equilibrium_pose, const std::string& info_string) {
    ROS_INFO("%s - Position (x, y, z): (%.2f, %.2f, %.2f), Orientation (x, y, z, w): (%.2f, %.2f, %.2f, %.2f)",
            info_string.c_str(),
            equilibrium_pose.pose.position.x, equilibrium_pose.pose.position.y, equilibrium_pose.pose.position.z,
            equilibrium_pose.pose.orientation.x, equilibrium_pose.pose.orientation.y, equilibrium_pose.pose.orientation.z, equilibrium_pose.pose.orientation.w);
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

  if (axis_map.find(fieldname) == axis_map.end() || joy_msg->axes.size() <= axis_map.at(fieldname))
  {
    return 0.0;
  }

  return joy_msg->axes[axis_map.at(fieldname)];
}

// Función para convertir un arreglo float64[16]/double en una matriz de transformación homogénea 4x4
Eigen::Affine3d convertFloat64ToAffine(const double* data) {
    Eigen::Matrix4d transformation_matrix;
    // Copia los datos del arreglo a la matriz de transformación
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            transformation_matrix(j, i) = data[i * 4 + j];
        }
    }
    // Convierte la matriz de transformación a una transformación afín 3D
    Eigen::Affine3d affine_transform(transformation_matrix);
    return affine_transform;
}

void TeleopFrankaJoy::Impl::obtainEquilibriumPose(const franka_msgs::FrankaStateConstPtr& msg){
  /*
  Función encargada de obtener el franka_msgs::FrankaState.O_T_EE y calcular el EquilibriumPose
  */

  current_franka_state = *msg; // Convierte de FrankaStateConstPtr [msg] a FrankaStatePtr [current_franka_state]

  // convert to eigen
  Eigen::Affine3d initial_transform = convertFloat64ToAffine(current_franka_state.O_T_EE.data());
  // set equilibrium point to current state
  Eigen::Vector3d position_d_ = initial_transform.translation();
  Eigen::Quaterniond orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());

  // Asignacion a equilibrium_pose
  geometry_msgs::PoseStamped calculated_equilibrium_pose;
  calculated_equilibrium_pose.pose.position.x = position_d_.x();
  calculated_equilibrium_pose.pose.position.y = position_d_.y();
  calculated_equilibrium_pose.pose.position.z = position_d_.z();

  calculated_equilibrium_pose.pose.orientation.x = orientation_d_.x();
  calculated_equilibrium_pose.pose.orientation.y = orientation_d_.y();
  calculated_equilibrium_pose.pose.orientation.z = orientation_d_.z();
  calculated_equilibrium_pose.pose.orientation.w = orientation_d_.w();

  equilibrium_pose = calculated_equilibrium_pose;

  printEquilibriumPoseInfo(calculated_equilibrium_pose, "Calculated Eq Pose");

}

double applyLimits(double value, double min_limit, double max_limit){
  // Aplica limites en el desplazamiento del equilibrium pose
  return std::min(std::max(value, min_limit), max_limit);
}

void TeleopFrankaJoy::Impl::ModifyVelocity(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                           float& scale,
                                           float& max_vel) {
    // Modifica la velocidad para una escala determinada
    if (joy_msg->buttons[increment_vel]) {

        //scale = scale*1.2; // Incremento de la escala
        scale = std::min(static_cast<double>(scale * 1.2), static_cast<double>(max_vel));
        ROS_INFO("Velocidad incrementada a %f", scale);

    } else if (joy_msg->buttons[decrement_vel]) {

        //scale = std::max(scale/1.2, min_vel); // Decremento de la escala
        scale = std::max(static_cast<double>(scale / 1.2), static_cast<double>(min_vel));
        ROS_INFO("Velocidad decrementada a %f", scale);

    }
    ros::Duration(reaction_t).sleep(); // Espera un tiempo de reaccion 
}


void TeleopFrankaJoy::Impl::sendCmdPositionMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_position_map)
  {
    geometry_msgs::Point increment_position;

    // Calculo incrementos
    increment_position.x = Delta_t * position_max_vel * getVal(joy_msg, axis_position_map, "x");
    increment_position.y = Delta_t * position_max_vel * getVal(joy_msg, axis_position_map, "y");
    increment_position.z = Delta_t * position_max_vel * getVal(joy_msg, axis_position_map, "z");

    //equilibrium_pose.header.seq++;
    equilibrium_pose.pose.position.x += increment_position.x;
    equilibrium_pose.pose.position.y += increment_position.y;
    equilibrium_pose.pose.position.z += increment_position.z;

    // Aplicamos límites a cada uno de los ejes
    equilibrium_pose.pose.position.x = applyLimits(equilibrium_pose.pose.position.x, -0.81, 0.81);
    equilibrium_pose.pose.position.y = applyLimits(equilibrium_pose.pose.position.y, -0.81, 0.81);
    equilibrium_pose.pose.position.z = applyLimits(equilibrium_pose.pose.position.z, -0.4, 1.15);

    cmd_PoseStamped_pub.publish(equilibrium_pose);

    printEquilibriumPoseInfo(equilibrium_pose, "Desired Eq pose");
    ros::Duration(Delta_t).sleep(); // Espera de Delta_t segundos
    ROS_INFO("Espera de Delta_t completada.");

  }

void TeleopFrankaJoy::Impl::sendCmdOrientationMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_orientation_map)
  {
    // ROS usa dos tipos de quaternions que no se peuden mezclar pero si convertir
    tf2::Quaternion q_rot;
    tf2::Quaternion q_orig;
    tf2::Quaternion q_new;

    // Convertir msg a tf2 para poder hacer (*)
    tf2::convert(equilibrium_pose.pose.orientation, q_orig);

    // Calculo de quaternion en funcion de angulos eulerXYZ
    double roll = Delta_t * orientation_max_vel * getVal(joy_msg, axis_orientation_map, "x");
    double pitch = Delta_t * orientation_max_vel * getVal(joy_msg, axis_orientation_map, "y");
    double yaw = Delta_t * orientation_max_vel * getVal(joy_msg, axis_orientation_map, "z");
    q_rot.setRPY(roll, pitch, yaw);

    // Aplicar rotacion
    q_new = q_rot * q_orig;
    
    // Normalizar
    q_new.normalize();

    // convertir de tf2 a msg
    tf2::convert(q_new, equilibrium_pose.pose.orientation);

    // Publicar nueva orientacion
    cmd_PoseStamped_pub.publish(equilibrium_pose);
    printEquilibriumPoseInfo(equilibrium_pose, "Desired Eq pose");

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
    ModifyVelocity(joy_msg, position_vel, position_max_vel);
    } else if(joy_msg->buttons[home_button]){ // LLeva a pto de reposo

      equilibrium_pose.pose.position.x = 0;
      equilibrium_pose.pose.position.y = 0.2;
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
      ModifyVelocity(joy_msg, orientation_vel, orientation_max_vel);

    } else if(joy_msg->buttons[home_button]){ // Pto de reposo angular
      
      equilibrium_pose.pose.orientation.x = 0;
      equilibrium_pose.pose.orientation.y = 0;
      equilibrium_pose.pose.orientation.z = 0;
      equilibrium_pose.pose.orientation.w = 1;

      cmd_PoseStamped_pub.publish(equilibrium_pose);

    }else{
      // Publicación segun el mando
      sendCmdOrientationMsg(joy_msg, axis_orientation_map);

    }  
  }else{ // Si no se toca nada
    
      cmd_PoseStamped_pub.publish(equilibrium_pose); // Se publica el equilibrium_pose cuando no se pulsa ninguna tecla
      printEquilibriumPoseInfo(equilibrium_pose, "Repose Eq Pose");

    // ros::Duration(Delta_t).sleep(); // Prueba: eliminar el temblor
  }
}

} // namespace teleop_franka_joy