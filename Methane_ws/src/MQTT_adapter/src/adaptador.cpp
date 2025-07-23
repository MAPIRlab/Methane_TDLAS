#include "MQTT_adapter/adaptador.hpp"
using std::placeholders::_1;

// Constructor de la clase adaptador
adaptador::adaptador() : Node("adaptador")
{
    // Crea un publicador para enviar mensajes tipo KeyValue al tópico /ros2mqtt
    Publicador_ros2mqtt = this -> create_publisher<diagnostic_msgs::msg::KeyValue>("/ros2mqtt",10);

    // Crea un suscriptor para recibir mensajes tipo KeyValue del tópico /mqtt2ros
    // Al recibir un mensaje, llama al método adaptar_publicacion
    Subscriptor_mqtt2ros = this -> create_subscription<diagnostic_msgs::msg::KeyValue>("/mqtt2ros",10,std::bind(&adaptador::adaptar_publicacion,this,_1));

    // Crea un publicador para enviar mensajes tipo String al tópico /initialize_hunter_params
    Publicador_inicializacion = this -> create_publisher<std_msgs::msg::String>("/initialize_hunter_params",10);

    // Crea un publicador para enviar mensajes tipo Bool al tópico /start_stop_hunter
    Publicador_comienzo_parada = this-> create_publisher<std_msgs::msg::Bool>("/start_stop_hunter",10);

    // Crea un suscriptor para recibir mensajes tipo NavSatFix del tópico /hunter/fix
    // Al recibir un mensaje, llama al método publica_pose
    Subscriptor_pose = this -> create_subscription<sensor_msgs::msg::NavSatFix>("/hunter/fix",10,std::bind(&adaptador::publica_pose,this,_1));
}

// Destructor de la clase adaptador
adaptador::~adaptador()
{
    // Muestra un mensaje de error al finalizar la ejecución
    RCLCPP_ERROR(this->get_logger(),"FIN DE LA EJECUCIÓN");
}

// Método para publicar la posición (latitud y longitud) en formato JSON
void adaptador::publica_pose(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    diagnostic_msgs::msg::KeyValue pose_keyvalue;
    pose_keyvalue.key = "/hunter_position";

    std::ostringstream ss;
    ss.precision(10);  // Ajusta la precisión a 10 decimales
    ss << std::fixed;
    ss << "{\"Latitude\": " << msg->latitude
    << ", \"Longitude\": " << msg->longitude << "}";

    pose_keyvalue.value = ss.str();

    // Publica el mensaje en el tópico /ros2mqtt
    Publicador_ros2mqtt -> publish(pose_keyvalue);
}

// Método para adaptar la publicación recibida desde MQTT y reenviarla al tópico correspondiente
void adaptador::adaptar_publicacion(const diagnostic_msgs::msg::KeyValue::SharedPtr msg)
{
    auto clave = msg->key;
    auto valor = msg->value;

    // Si la clave es /initialize_hunter_params, publica el valor como String
    if(clave == "/initialize_hunter_params")
    {
        std_msgs::msg::String mensaje;
        mensaje.data = valor;
        Publicador_inicializacion -> publish(mensaje);
    }
    // Si la clave es /start_stop_hunter, publica el valor como Bool
    else if (clave == "/start_stop_hunter")
    {
        std_msgs::msg::Bool mensaje;
        if (valor == "True")
        {
            mensaje.data = true;
        }
        else
        {
            mensaje.data = false;
        }
        Publicador_comienzo_parada -> publish (mensaje);
    }
}

// Función principal del programa
int main(int argc, char *argv[])
{ 
    rclcpp::init(argc, argv);
    auto node = std::make_shared<adaptador>();

    // Configura el nivel de log a Info
    auto logger = node->get_logger();
    logger.set_level(rclcpp::Logger::Level::Info);  

    // Bucle principal: ejecuta el nodo a 100 Hz
    rclcpp::Rate loop_rate(100.00);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
