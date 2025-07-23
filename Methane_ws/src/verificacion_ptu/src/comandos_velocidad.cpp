#include "verificacion_ptu/comandos_velocidad.hpp"

// Constructor de la clase comandos_velocidad
comandos_velocidad::comandos_velocidad() : Node("comandos_velocidad")
{
    // Se crea un publicador para el tópico /cmd_vel con un tamaño de cola de 10
    Publicador = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Se declaran los parámetros 'v' (velocidad lineal) y 'R' (radio de giro) con valor por defecto 0.0
    this->declare_parameter<double>("v",0.0);
    this->declare_parameter<double>("R",0.0);

    // Se obtienen los valores de los parámetros y se asignan a las variables miembro
    v = this->get_parameter("v").as_double();
    R = this->get_parameter("R").as_double();
}

// Destructor de la clase comandos_velocidad
comandos_velocidad::~comandos_velocidad()
{
    // Mensaje de error al salir de la ejecución
    RCLCPP_ERROR(this->get_logger(), "SALIENDO DE LA EJECUCIÓN");
}

// Método para enviar comandos de velocidad al robot
void comandos_velocidad::comandar_hunter()
{
    geometry_msgs::msg::Twist accion;
    // Se calcula la velocidad angular w = v / R
    w = v / R;
    accion.linear.x = v;
    accion.angular.z = w;

    // Se imprime en consola los valores de v y R
    RCLCPP_INFO(this->get_logger(),"LOS VALORES DE V: %f Y DE R: %f",v,R);

    // Se publica el mensaje en el tópico /cmd_vel
    Publicador->publish(accion);
}

// Función principal del programa
int main(int argc, char *argv[])
{
    // Inicializa ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<comandos_velocidad>();

    // Configura el nivel de log en Info
    auto logger = node->get_logger();
    logger.set_level(rclcpp::Logger::Level::Info);

    // Se define una tasa de ejecución de 10 Hz
    rclcpp::Rate loop_rate(10.000);
    while (rclcpp::ok())
    {
        // Procesa callbacks pendientes
        rclcpp::spin_some(node);
        // Espera para mantener la frecuencia
        loop_rate.sleep();
        // Llama al método para enviar comandos de velocidad
        node->comandar_hunter();
    }

    // Finaliza ROS2
    rclcpp::shutdown();
    return 0;
}