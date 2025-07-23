#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nlohmann/json.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using json = nlohmann::json;

/**
 * @class simulacion_movimiento
 * @brief Nodo de ROS2 para simular el movimiento de un sistema.
 *
 * Esta clase hereda de rclcpp::Node y permite simular el movimiento de un sistema
 * utilizando ROS2. Publica y suscribe mensajes relevantes para la simulación.
 *
 * Métodos públicos:
 * - simulacion_movimiento(): Constructor de la clase.
 * - ~simulacion_movimiento(): Destructor de la clase.
 * - void simulacion(float w_ptu): Ejecuta la simulación del movimiento con el parámetro w_ptu.
 * - void callback_estado(const sensor_msgs::msg::JointState::SharedPtr msg): Callback para procesar el estado recibido.
 *
 * Atributos públicos:
 * - rclcpp::Time tiempo_anterior: Almacena el tiempo anterior para cálculos de simulación.
 *
 * Atributos privados:
 * - rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Publicador: Publicador de mensajes de tipo String.
 * - rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr Subscriptor_Info: Suscriptor de mensajes JointState.
 * - int width, height: Dimensiones del entorno simulado.
 * - float d, v, eq_x, pos_x_real, pos_y_real, angulo_actual: Variables para la simulación de movimiento.
 * - std::chrono::time_point<std::chrono::steady_clock> tiempo_inicio: Marca de tiempo de inicio de la simulación.
 */
class simulacion_movimiento : public rclcpp::Node
{
    public:
        simulacion_movimiento();
        ~simulacion_movimiento();
        void simulacion(float w_ptu);
        void callback_estado(const sensor_msgs::msg::JointState::SharedPtr msg);

        rclcpp::Time tiempo_anterior;

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Publicador;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr Subscriptor_Info;

        int width, height;
        float d, v, eq_x, pos_x_real, pos_y_real, angulo_actual;
        std::chrono::time_point<std::chrono::steady_clock> tiempo_inicio;
};