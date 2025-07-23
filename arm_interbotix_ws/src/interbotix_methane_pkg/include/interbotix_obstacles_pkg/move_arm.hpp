#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

/**
 * @class MoveArmNode
 * @brief Nodo de ROS 2 para controlar el brazo robótico mediante la recepción de poses objetivo.
 *
 * Esta clase implementa un nodo que suscribe a mensajes de tipo geometry_msgs::msg::Pose
 * y permite establecer la pose del brazo robótico a través del método set_arm_pose.
 *
 * Métodos públicos:
 *  - MoveArmNode(): Constructor del nodo.
 *  - ~MoveArmNode(): Destructor del nodo.
 *  - void set_arm_pose(const geometry_msgs::msg::Pose::SharedPtr msg): 
 *      Establece la pose del brazo robótico según el mensaje recibido.
 *
 * Atributos privados:
 *  - sub_pose: Suscripción a mensajes de tipo geometry_msgs::msg::Pose.
 */
class MoveArmNode : public rclcpp::Node
{
public:
    MoveArmNode();
    ~MoveArmNode();
    void set_arm_pose(const geometry_msgs::msg::Pose::SharedPtr msg);
private:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_pose;
};