#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

/**
 * @class comandos_velocidad
 * @brief Nodo de ROS2 encargado de comandar la velocidad de un robot tipo Hunter.
 *
 * Esta clase hereda de rclcpp::Node y permite publicar mensajes de velocidad
 * (Twist) para controlar el movimiento del robot. Incluye métodos para inicializar
 * el nodo, limpiar recursos y enviar comandos de velocidad.
 *
 * @author Tu Nombre
 * @date Fecha de creación
 */
class comandos_velocidad : public rclcpp::Node
{
    public:
    comandos_velocidad();
    ~comandos_velocidad();
    void comandar_hunter();

    private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Publicador;

    double v,R,w;
};