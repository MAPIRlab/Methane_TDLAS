#include "interbotix_obstacles_pkg/map_obstacles.hpp"

// Constructor del nodo AddObstacleNode
AddObstacleNode::AddObstacleNode()
: Node("add_obstacle_node")
{
  define_obstacles(); // Llama a la función para definir los obstáculos al inicializar el nodo
}

// Función para definir y añadir todos los obstáculos al entorno
void AddObstacleNode::define_obstacles()
{
  std::vector<double> dimensions;
  std::vector<double> position;
  std_msgs::msg::String id;

  // Base_Hunter
  dimensions = {0.4, 0.95, 0.39}; // Dimensiones del obstáculo (largo, ancho, alto)
  position = {0.0, 0.31, 0.2};    // Posición del obstáculo en el mundo
  id.data = "Hunter_Base";         // Identificador del obstáculo
  addObstacle(id, dimensions, position);

  // Rueda derecha-atras
  dimensions = {0.17, 0.3, 0.3};
  position = {0.2+0.17/2, 0, 0.15};
  id.data = "Hunter_Wheel_Right_Back";
  addObstacle(id, dimensions, position);

  // Varilla de aluminio de atrás
  dimensions = {0.51, 0.02, 0.02};
  position = {0, -0.13, 0.40};
  id.data = "Back_Aluminium_profile";
  addObstacle(id, dimensions, position);

  // Tope izquierda varilla de aluminio
  dimensions = {0.02, 0.045, 0.02};
  position = {-0.26, -0.12-0.045/2, 0.40};
  id.data = "Left_Aluminium_profile";
  addObstacle(id, dimensions, position);

  // Tope derecha varilla de aluminio
  dimensions = {0.02, 0.045, 0.02};
  position = {+0.26, -0.12-0.045/2, 0.40};
  id.data = "Right_Aluminium_profile";
  addObstacle(id, dimensions, position);

  // GPS
  dimensions = {0.12, 0.12, 0.18};
  position = {0.0, -0.11-0.12/2, 0.39+0.18/2+0.02};
  id.data = "GPS";
  addObstacle(id, dimensions, position);

  // Screen
  dimensions = {0.45, 0.1, 0.2};
  position = {0.0, 0.20, 0.39+0.2/2};
  id.data = "Screen";
  addObstacle(id, dimensions, position);

  // Estructura superior
  dimensions = {0.28, 0.22, 0.29};
  position = {0.0, 0.2+0.05+0.11, 0.39+0.29/2};
  id.data = "Upper_Structure";
  addObstacle(id, dimensions, position);

  // OUSTER
  dimensions = {0.08, 0.22, 0.1};
  position = {0.0, 0.2+0.05+0.11, 0.39+0.29+0.05};
  id.data = "OUSTER";
  addObstacle(id, dimensions, position);

  RCLCPP_INFO(this->get_logger(), "Añadidos todos los obstáculos"); // Mensaje informativo
}

// Función para añadir un obstáculo individual al entorno de MoveIt
void AddObstacleNode::addObstacle(std_msgs::msg::String id, std::vector<double> dimensions, std::vector<double> position)
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "world"; // El marco de referencia es "world"
  collision_object.id = id.data;              // Asigna el identificador

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;             // Tipo de primitiva: caja
  primitive.dimensions = {dimensions[0], dimensions[1], dimensions[2]}; // Dimensiones de la caja

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;               // Orientación por defecto (sin rotación)
  box_pose.position.x = position[0];
  box_pose.position.y = position[1];
  box_pose.position.z = position[2];

  collision_object.primitives.push_back(primitive);         // Añade la primitiva al objeto de colisión
  collision_object.primitive_poses.push_back(box_pose);     // Añade la pose
  collision_object.operation = collision_object.ADD;        // Operación: añadir

  planning_scene_interface_.applyCollisionObjects({collision_object}); // Aplica el objeto de colisión al entorno

  RCLCPP_INFO(this->get_logger(), "Added obstacle: %s", id.data.c_str()); // Mensaje informativo
}

// Función principal del nodo
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);                        // Inicializa ROS2
  auto node = std::make_shared<AddObstacleNode>(); // Crea el nodo
  rclcpp::spin(node);                              // Ejecuta el nodo
  rclcpp::shutdown();                              // Finaliza ROS2
  return 0;
}
