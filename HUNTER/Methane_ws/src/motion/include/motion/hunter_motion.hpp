#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "nlohmann/json.hpp"
#include <GeographicLib/UTMUPS.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

/**
 * @class hunter_motion
 * @brief Nodo principal para el control de movimiento y orientación de un robot tipo Hunter en ROS2.
 *
 * Esta clase gestiona la trayectoria, la orientación del brazo y la inicialización del robot,
 * así como la recepción y procesamiento de mensajes relacionados con el movimiento y la posición.
 *
 * Métodos públicos:
 * - hunter_motion(): Constructor de la clase.
 * - ~hunter_motion(): Destructor de la clase.
 * - void hunter_trajectory(): Calcula y ejecuta la trayectoria del robot.
 * - void arm_orientation(): Controla la orientación del brazo del robot.
 * - void initialize(const std_msgs::msg::String::SharedPtr msg): Inicializa el sistema a partir de un mensaje recibido.
 * - void enable_function(const std_msgs::msg::Bool::SharedPtr msg): Habilita o deshabilita funciones según el mensaje recibido.
 * - void process_pose(const sensor_msgs::msg::NavSatFix::SharedPtr msg): Procesa la posición GPS recibida.
 *
 * Atributos privados:
 * - Publisher_vel: Publicador de velocidades lineales y angulares (Twist).
 * - Publisher_arm_pose: Publicador de la pose del brazo (Pose).
 * - Publisher_end_hunter: Publicador para indicar el fin de la operación.
 * - Subscriber_initialize: Suscriptor para mensajes de inicialización.
 * - Subscriber_start_stop: Suscriptor para mensajes de inicio/parada.
 * - Subscriber_pose: Suscriptor para mensajes de posición GPS.
 * - max_vel, lookahead: Parámetros de control de velocidad y anticipación.
 * - latitude_ptu, longitude_ptu, x_ptu, y_ptu: Coordenadas de referencia para el PTU.
 * - enable: Indica si el sistema está habilitado.
 * - path_points: Vector de puntos (latitud, longitud) que conforman la trayectoria.
 * - x_hunter, y_hunter, theta_hunter: Estado actual del robot (posición y orientación).
 * - zone: Zona actual del robot.
 * - northp: Indica si el robot se encuentra en el hemisferio norte.
 * - primera_vez_brazo, inicializacion_brazo: Flags para el control del brazo robótico.
 */
class hunter_motion : public rclcpp::Node
{
    public:
    hunter_motion();
    ~hunter_motion();
    void hunter_trajectory();
    void arm_orientation();
    void initialize(const std_msgs::msg::String::SharedPtr msg);
    void enable_function(const std_msgs::msg::Bool::SharedPtr msg);
    void process_pose(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Publisher_vel;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr Publisher_arm_pose;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Publisher_end_hunter;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Subscriber_initialize;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Subscriber_start_stop;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr Subscriber_pose;

    float max_vel, lookahead;
    double latitude_ptu, longitude_ptu, x_ptu, y_ptu;
    bool enable;
    std::vector<std::pair<double, double>> path_points;

    double x_hunter, y_hunter, theta_hunter;

    int zone;
    bool northp;
    bool primera_vez_brazo, inicializacion_brazo;
};