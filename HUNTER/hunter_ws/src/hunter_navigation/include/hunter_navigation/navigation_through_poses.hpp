#include "rclcpp/rclcpp.hpp" // Incluye la funcionalidad principal de ROS2 en C++
#include "geometry_msgs/msg/pose.hpp" // Mensaje para representar una pose (posición y orientación)
#include "nav2_msgs/action/navigate_to_pose.hpp" // Acción para navegar a una pose específica
#include "nlohmann/json.hpp" // Biblioteca para manejar JSON en C++
#include "std_msgs/msg/bool.hpp" // Mensaje estándar booleano de ROS2
#include "rclcpp_action/rclcpp_action.hpp" // Soporte para acciones en ROS2
#include "olfaction_msgs/msg/tdlas.hpp" // Mensaje personalizado para sensores de olfacción (TDLAS)

using NavigateToPose = nav2_msgs::action::NavigateToPose; // Alias para simplificar el uso de la acción
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>; // Alias para el manejador de objetivos de la acción

using json = nlohmann::json; // Alias para la clase json de la biblioteca nlohmann

// Definición de la clase principal que hereda de rclcpp::Node
class navigation_through_poses : public rclcpp::Node
{
    public:
    navigation_through_poses(); // Constructor
    ~navigation_through_poses(); // Destructor

    // Callback que se ejecuta cuando se recibe un mensaje booleano para mover el PTU
    void callback_movimiento_ptu(const std_msgs::msg::Bool::SharedPtr msg);

    // Callback que se ejecuta cuando termina la acción de navegación
    void callback_movimiento_hunter(const GoalHandleNavigateToPose::WrappedResult &result);

    // Callback que se ejecuta al recibir datos del sensor TDLAS
    void callback_TDLAS(const olfaction_msgs::msg::TDLAS::SharedPtr msg);

    private:

    // Matriz de poses: cada habitación tiene una lista de poses a las que navegar
    std::vector<std::vector<geometry_msgs::msg::Pose>> poses_;

    // Publicador para indicar el inicio del movimiento del PTU
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Publicador_inicio_ptu;

    // Suscriptor para recibir comandos de movimiento del PTU
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Subscriptor_ptu;

    // Suscriptor para recibir datos del sensor TDLAS
    rclcpp::Subscription<olfaction_msgs::msg::TDLAS>::SharedPtr Subscriptor_TDLAS;

    // Cliente de acción para enviar objetivos de navegación
    rclcpp_action::Client<NavigateToPose>::SharedPtr cliente_poses;

    // Índices para llevar el control de la habitación y el punto actual
    int indice_habitacion, indice_punto;

    // Variables de estado para saber si hay metano en la habitación y si se está escaneando con el PTU
    bool habitacion_con_metano, escaneo_ptu;
};