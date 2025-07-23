#include "rclcpp/rclcpp.hpp"
#include "interbotix_xs_msgs/msg/joint_single_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "interbotix_xs_msgs/srv/operating_modes.hpp"

/**
 * @class scanner_ptu
 * @brief Nodo ROS2 para controlar un escáner PTU (Pan-Tilt Unit) en un robot Interbotix.
 *
 * Esta clase gestiona la operación de un escáner PTU, permitiendo iniciar el escaneo,
 * recibir el estado de las articulaciones y controlar el ciclo principal de escaneo.
 * Utiliza suscripciones y publicaciones ROS2 para interactuar con otros nodos y servicios.
 *
 * Hereda de rclcpp::Node para integrarse en el sistema ROS2 y de std::enable_shared_from_this
 * para facilitar la gestión de memoria compartida.
 *
 * Métodos públicos:
 *  - scanner_ptu(): Constructor de la clase.
 *  - ~scanner_ptu(): Destructor de la clase.
 *  - void start_scanner(const std_msgs::msg::Bool::SharedPtr msg): Inicia el escaneo al recibir un mensaje.
 *  - void callback_estado(const sensor_msgs::msg::JointState::SharedPtr msg): Callback para actualizar el estado de las articulaciones.
 *  - void main_loop(): Ejecuta el ciclo principal de operación del escáner.
 *
 * Atributos privados:
 *  - Subscriptor_start: Suscripción para iniciar el escaneo.
 *  - Subscriptor_Info: Suscripción para recibir información del estado de las articulaciones.
 *  - Publicador_vel: Publicador para enviar comandos de velocidad a la PTU.
 *  - Publicador_fin: Publicador para indicar el fin del escaneo.
 *  - client_: Cliente para cambiar los modos de operación de la PTU.
 *  - position_pan_ptu, position_tilt_ptu: Posiciones actuales de los ejes pan y tilt.
 *  - angulo_inicio, angulo_fin: Ángulos de inicio y fin del escaneo.
 *  - escaneo, recuperacion, inicializacion_pan, inicializacion_tilt, inicializacion: Banderas de control del estado del escáner.
 */
class scanner_ptu : public rclcpp::Node, public std::enable_shared_from_this<scanner_ptu>
{
    public:
    scanner_ptu();
    ~scanner_ptu();
    void start_scanner(const std_msgs::msg::Bool::SharedPtr msg);
    void callback_estado(const sensor_msgs::msg::JointState::SharedPtr msg);
    void main_loop();

    private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Subscriptor_start;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr Subscriptor_Info;
    rclcpp::Publisher<interbotix_xs_msgs::msg::JointSingleCommand>::SharedPtr Publicador_vel;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Publicador_fin;

    rclcpp::Client<interbotix_xs_msgs::srv::OperatingModes>::SharedPtr client_;

    double position_pan_ptu, position_tilt_ptu, angulo_inicio, angulo_fin;
    bool escaneo, recuperacion, inicializacion_pan, inicializacion_tilt, inicializacion;
};