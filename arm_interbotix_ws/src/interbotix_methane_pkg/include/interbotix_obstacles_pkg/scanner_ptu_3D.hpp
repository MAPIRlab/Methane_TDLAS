#include "rclcpp/rclcpp.hpp"
#include "interbotix_xs_msgs/msg/joint_single_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "interbotix_xs_msgs/srv/operating_modes.hpp"

// Clase scanner_ptu_3D:
// Esta clase hereda de rclcpp::Node y permite controlar un escáner 3D montado en una PTU (Pan-Tilt Unit).
// Proporciona métodos para iniciar el escaneo, manejar el estado de la PTU y ejecutar la secuencia principal de escaneo.
//
// Métodos públicos:
// - scanner_ptu_3D(): Constructor de la clase.
// - ~scanner_ptu_3D(): Destructor de la clase.
// - void start_scanner(const std_msgs::msg::Bool::SharedPtr msg): Callback para iniciar el escaneo al recibir un mensaje.
// - void callback_estado(const sensor_msgs::msg::JointState::SharedPtr msg): Callback para actualizar el estado de la PTU.
// - void main_loop(): Método principal que ejecuta el bucle de control del escáner.
// - void secuencia_escaneo(): Ejecuta la secuencia de escaneo 3D.
//
// Atributos privados:
// - Subscriptor_start: Suscriptor para mensajes de inicio de escaneo.
// - Subscriptor_Info: Suscriptor para mensajes de estado de la PTU.
// - Publicador_vel: Publicador para comandos de velocidad de la PTU.
// - Publicador_fin: Publicador para indicar el fin del escaneo.
// - client_: Cliente para cambiar los modos de operación de la PTU.
// - position_pan_ptu, position_tilt_ptu: Variables para almacenar la posición actual de los ejes pan y tilt.
// - angulo_inicio, angulo_fin: Ángulos de inicio y fin del escaneo.
// - escaneo, recuperacion, inicializacion_pan, inicializacion_tilt, inicializacion, tramo1, tramo2, tramo3, tramo4, tramo5, tramo6, tramo7: Banderas de control para el estado del escaneo y las diferentes etapas de la secuencia.
class scanner_ptu_3D : public rclcpp::Node, public std::enable_shared_from_this<scanner_ptu_3D>
{
    public:
    scanner_ptu_3D();
    ~scanner_ptu_3D();
    void start_scanner(const std_msgs::msg::Bool::SharedPtr msg);
    void callback_estado(const sensor_msgs::msg::JointState::SharedPtr msg);
    void main_loop();
    void secuencia_escaneo();

    private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Subscriptor_start;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr Subscriptor_Info;
    rclcpp::Publisher<interbotix_xs_msgs::msg::JointSingleCommand>::SharedPtr Publicador_vel;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Publicador_fin;

    rclcpp::Client<interbotix_xs_msgs::srv::OperatingModes>::SharedPtr client_;

    double position_pan_ptu, position_tilt_ptu, angulo_inicio, angulo_fin;
    bool escaneo, recuperacion, inicializacion_pan, inicializacion_tilt, inicializacion, tramo1, tramo2, tramo3, tramo4, tramo5, tramo6, tramo7;
};