#include "interbotix_obstacles_pkg/move_arm.hpp"
using std::placeholders::_1;

// Constructor del nodo MoveArmNode
MoveArmNode::MoveArmNode() : Node("move_arm_node")
{
    // Se crea una suscripción al tópico "/arm_pose" para recibir mensajes de tipo geometry_msgs::msg::Pose
    // Cuando se recibe un mensaje, se llama al método set_arm_pose
    sub_pose = this->create_subscription<geometry_msgs::msg::Pose>(
        "/arm_pose", 10, std::bind(&MoveArmNode::set_arm_pose, this, std::placeholders::_1));
}

// Destructor del nodo MoveArmNode
MoveArmNode::~MoveArmNode()
{
    // Se imprime un mensaje de error al finalizar el movimiento
    RCLCPP_ERROR(this->get_logger(), "FIN DEL MOVIMIENTO");
}

// Método que se ejecuta al recibir un mensaje con la pose objetivo del brazo
void MoveArmNode::set_arm_pose(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    // Se informa que se ha recibido un mensaje
    RCLCPP_INFO(this->get_logger(),"MENSAJE RECIBIDO");

    // Se crea una instancia de MoveGroupInterface para controlar el brazo "interbotix_arm"
    moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(),"interbotix_arm");

    // Se muestra el marco de referencia de la pose
    RCLCPP_INFO(this->get_logger(), "Pose reference frame: %s", move_group.getPoseReferenceFrame().c_str());

    // Se establece la pose objetivo recibida
    move_group.setPoseTarget(*msg);

    // Se planea el movimiento hacia la pose objetivo
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        // Si la planificación fue exitosa, se ejecuta el movimiento
        RCLCPP_INFO(this->get_logger(), "Planning successful, moving to target...");
        move_group.execute(my_plan);
    }
    else
    {
        // Si la planificación falla, se muestra una advertencia
        RCLCPP_WARN(this->get_logger(), "Planning failed."); 
    }
}

// Función principal del programa
int main(int argc, char** argv)
{
  // Inicializa ROS2
  rclcpp::init(argc, argv);

  // Crea una instancia del nodo MoveArmNode
  auto node = std::make_shared<MoveArmNode>();

  // Mantiene el nodo activo para recibir mensajes
  rclcpp::spin(node);

  // Finaliza ROS2
  rclcpp::shutdown();
  return 0;
}
