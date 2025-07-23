#include "rclcpp/rclcpp.hpp"
#include "interbotix_xs_msgs/msg/joint_single_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "interbotix_xs_msgs/srv/operating_modes.hpp"

// Definición de la clase scanner_ptu_2, que hereda de rclcpp::Node y permite el uso de shared_ptr
class scanner_ptu_2 : public rclcpp::Node, public std::enable_shared_from_this<scanner_ptu_2>
{
    public:
    // Constructor de la clase
    scanner_ptu_2();
    // Destructor de la clase
    ~scanner_ptu_2();
    // Método para iniciar el escáner, recibe un mensaje Bool
    void start_scanner(const std_msgs::msg::Bool::SharedPtr msg);
    // Callback que procesa el estado de las articulaciones
    void callback_estado(const sensor_msgs::msg::JointState::SharedPtr msg);
    // Bucle principal de la lógica del escáner
    void main_loop();

    private:
    // Suscripción para recibir mensajes que indican cuándo iniciar el escaneo
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Subscriptor_start;
    // Suscripción para recibir el estado de las articulaciones
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr Subscriptor_Info;
    // Publicador para enviar comandos de velocidad a una articulación
    rclcpp::Publisher<interbotix_xs_msgs::msg::JointSingleCommand>::SharedPtr Publicador_vel;
    // Publicador para indicar que el escaneo ha finalizado
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Publicador_fin;

    // Cliente para llamar al servicio de modos de operación del robot
    rclcpp::Client<interbotix_xs_msgs::srv::OperatingModes>::SharedPtr client_;

    // Variables para almacenar la posición actual del pan y tilt, y los ángulos de inicio y fin
    double position_pan_ptu, position_tilt_ptu, angulo_inicio, angulo_fin;
    // Variables de estado para controlar el flujo del escaneo y la inicialización
    bool escaneo, recuperacion, inicializacion_pan, inicializacion_tilt, inicializacion;
};