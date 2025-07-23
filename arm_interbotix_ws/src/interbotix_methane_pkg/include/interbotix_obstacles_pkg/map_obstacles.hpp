#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometric_shapes/solid_primitive_dims.h>
#include <std_msgs/msg/string.hpp>

/* Este nodo se utiliza en MoveIt2 para añadir objetos de colisión/obstáculos en la escena
útiles para la planificación de trayectorias evitando colisiones*/
class AddObstacleNode : public rclcpp::Node
{
public:
  AddObstacleNode();

private:
  void addObstacle(std_msgs::msg::String id, std::vector<double> dimensions, std::vector<double> position);
  void define_obstacles();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};