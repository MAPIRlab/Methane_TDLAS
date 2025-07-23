#include "percepcion/simulacion_movimiento.hpp"

using std::placeholders::_1;


simulacion_movimiento::simulacion_movimiento() : Node("simulacion_movimiento")
{
    Publicador = this->create_publisher<std_msgs::msg::String>("/Info_Posicion", 10);
    Subscriptor_Info = this ->create_subscription<sensor_msgs::msg::JointState>("/PTU_Methane/joint_states", 10, std::bind(&simulacion_movimiento::callback_estado,this,_1));
    width = 1280;
    height = 720;

    d = 20.0;
    v = 1.0;
    eq_x = 0.00096;

    tiempo_anterior = now();
}

simulacion_movimiento :: ~simulacion_movimiento()
{
    RCLCPP_ERROR(this->get_logger(), "FIN DE LA SIMULACION");
}

void simulacion_movimiento::callback_estado(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    float w_ptu;

    w_ptu=msg->velocity[0];
    w_ptu=w_ptu/eq_x;

    RCLCPP_INFO(this->get_logger(),"Velocidad de seguimiento PTU: %f px/s", w_ptu);
    simulacion(w_ptu);
}

/**
 * @brief Simulates the movement of an object and publishes its position.
 *
 * This method updates the simulated position of an object based on the provided PTU (Pan-Tilt Unit) angular velocity,
 * internal velocity parameters, and elapsed time. The updated position is published as a JSON-formatted string message.
 *
 * @param w_ptu The angular velocity input from the PTU, in pixels per second.
 *
 * The function performs the following steps:
 * - Calculates the elapsed time since the last update.
 * - Updates the object's X position based on the combined velocity and PTU input.
 * - Keeps the Y position constant.
 * - Publishes the updated position and simulation parameters as a JSON message.
 *
 * @note The function uses static variables to maintain state between calls.
 */
void simulacion_movimiento :: simulacion(float w_ptu)
{
    static float pos_x = width/2;
    static float pos_y = height/2;
    float w, w_pix;
    json dato;
    dato["width"] = width;
    dato["height"] = height;

    w=v/d;
    w_pix=w/eq_x;

    rclcpp::Time tiempo_actual = now();
    float dt=(tiempo_actual - tiempo_anterior).seconds();

    if (dt<0.000000001)
    {
        dt=0.1;
    }

    pos_x = pos_x +  (w_pix + w_ptu) * dt;
    tiempo_anterior = tiempo_actual;
    pos_y = pos_y;

    RCLCPP_INFO(this->get_logger(),"POS DESEADA: %f", pos_x);

    dato["pos_x"] = pos_x;
    dato["pos_y"] = pos_y;

    std::string dato_str = dato.dump();
    auto msg = std_msgs::msg::String();
    msg.data = dato_str;
    Publicador->publish(msg);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<simulacion_movimiento>(); // Reemplaza "MiNodo" con el nombre de tu clase de nodo

    while (rclcpp::ok()) {
        rclcpp::spin_some(node); // Procesa los mensajes del topic lo más rápido posible
    }

    rclcpp::shutdown();
    return 0;
}