// Inclusión de las cabeceras necesarias de ROS2 y mensajes estándar
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

// Definición de la clase 'adaptador', que hereda de rclcpp::Node
class adaptador : public rclcpp::Node
{
    public:
    // Constructor de la clase
    adaptador();
    // Destructor de la clase
    ~adaptador();
    // Método para adaptar y publicar mensajes de tipo KeyValue
    void adaptar_publicacion(const diagnostic_msgs::msg::KeyValue::SharedPtr msg);
    // Método para publicar la posición (pose) usando mensajes NavSatFix
    void publica_pose(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    private:
    // Publicador para enviar mensajes KeyValue de ROS2 a MQTT
    rclcpp::Publisher<diagnostic_msgs::msg::KeyValue>::SharedPtr Publicador_ros2mqtt;
    // Suscriptor para recibir mensajes KeyValue de MQTT a ROS2
    rclcpp::Subscription<diagnostic_msgs::msg::KeyValue>::SharedPtr Subscriptor_mqtt2ros;
    // Publicador para enviar mensajes de inicialización (tipo String)
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Publicador_inicializacion;
    // Publicador para enviar mensajes de comienzo/parada (tipo Bool)
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Publicador_comienzo_parada;
    // Suscriptor para recibir mensajes de posición (NavSatFix)
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr Subscriptor_pose;
};