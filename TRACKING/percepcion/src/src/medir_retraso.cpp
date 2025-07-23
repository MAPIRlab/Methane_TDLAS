#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <interbotix_xs_msgs/msg/joint_group_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

/**
 * @class MedirRetrasoPTU
 * @brief Nodo ROS2 para medir el retraso de respuesta de una PTU (Pan-Tilt Unit).
 *
 * Esta clase implementa un nodo que envía comandos de movimiento mínimos a la PTU y mide el tiempo
 * que tarda en recibir una respuesta a través del tópico de estados de las articulaciones. El retraso
 * medido se publica en un tópico dedicado. El nodo también detiene la PTU después de cada medición.
 *
 * Funcionalidades principales:
 * - Publica comandos de movimiento a la PTU en el tópico "/PTU_Methane/commands/joint_group".
 * - Suscribe al tópico "/PTU_Methane/joint_states" para recibir el estado de la PTU.
 * - Publica el retraso medido en el tópico "/ptu_delay".
 * - Utiliza un temporizador para enviar comandos periódicamente (cada 2 segundos).
 *
 * Variables privadas:
 * - comando_pub_: Publicador de comandos para la PTU.
 * - respuesta_sub_: Suscriptor de estados de la PTU.
 * - retraso_pub_: Publicador del retraso medido.
 * - timer_: Temporizador para el envío periódico de comandos.
 * - tiempo_envio_: Marca de tiempo del envío del comando.
 * - esperando_respuesta_: Indica si se está esperando una respuesta de la PTU.
 *
 * Métodos privados:
 * - enviarComando(): Envía un comando de movimiento mínimo a la PTU y registra el tiempo de envío.
 * - callbackRespuestaPTU(): Procesa la respuesta de la PTU, calcula el retraso, publica el resultado y detiene la PTU.
 */
class MedirRetrasoPTU : public rclcpp::Node
{
public:
    MedirRetrasoPTU() : Node("medir_retraso_ptu"), esperando_respuesta_(false)
    {
        // Publicador de comandos a la PTU
        comando_pub_ = this->create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>(
            "/PTU_Methane/commands/joint_group", 10);
        
        // Suscriptor de la respuesta de la PTU
        respuesta_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/PTU_Methane/joint_states", 10,
            std::bind(&MedirRetrasoPTU::callbackRespuestaPTU, this, std::placeholders::_1));

        // Publicador del retraso
        retraso_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ptu_delay", 10);

        // Temporizador para enviar comandos cada 2 segundos
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2), std::bind(&MedirRetrasoPTU::enviarComando, this));

        RCLCPP_INFO(this->get_logger(), "Nodo de medición de retraso iniciado.");
    }

private:
    rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr comando_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr respuesta_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr retraso_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Time tiempo_envio_;
    bool esperando_respuesta_;

    void enviarComando()
    {
        if (esperando_respuesta_)
            return; // No enviar otro comando hasta procesar la respuesta anterior

        auto msg = interbotix_xs_msgs::msg::JointGroupCommand();
        msg.name = "turret";
        msg.cmd = {0.1, -0.1}; // Movimiento mínimo para medir retraso

        tiempo_envio_ = now();
        comando_pub_->publish(msg);
        esperando_respuesta_ = true;

        RCLCPP_INFO(this->get_logger(), "Comando enviado a la PTU.");
    }

    void callbackRespuestaPTU(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!esperando_respuesta_)
            return; // Solo procesar si está esperando una respuesta

        rclcpp::Time tiempo_respuesta = now();
        double retraso = (tiempo_respuesta - tiempo_envio_).seconds();

        auto retraso_msg = std_msgs::msg::Float64();
        retraso_msg.data = retraso;
        retraso_pub_->publish(retraso_msg);

        RCLCPP_INFO(this->get_logger(), "Retraso medido: %.4f segundos", retraso);

        // Enviar comando de parada (0.0, 0.0)
        auto stop_msg = interbotix_xs_msgs::msg::JointGroupCommand();
        stop_msg.name = "turret";
        stop_msg.cmd = {0.0, 0.0};
        comando_pub_->publish(stop_msg);

        RCLCPP_INFO(this->get_logger(), "PTU detenida.");
        
        // Resetear para permitir el próximo comando
        esperando_respuesta_ = false;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MedirRetrasoPTU>());
    rclcpp::shutdown();
    return 0;
}
