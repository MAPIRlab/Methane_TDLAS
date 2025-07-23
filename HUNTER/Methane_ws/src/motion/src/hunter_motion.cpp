#include "motion/hunter_motion.hpp"
#include <cmath>
using std::placeholders::_1;
using json = nlohmann::json;

// DEFINICIÓN DE LOS MÉTODOS DE LA CLASE HUNTER_MOTION
hunter_motion::hunter_motion() : Node("hunter_motion")
{
    // Definición del publicador que va a comandar al robot
    Publisher_vel = this -> create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);

    // Definición del publicador de poses del brazo
    Publisher_arm_pose = this -> create_publisher<geometry_msgs::msg::Pose>("/arm_pose",10);

    // Definición del publicador de fin de movimiento del Hunter
    Publisher_end_hunter = this -> create_publisher<std_msgs::msg::Bool>("/end_hunter",10);
    
    // Definición de los subscriptores que van a permitir en orden de definición:
    // 1. Inicializar los parámetros del robot, con la trayectoria a seguir, la velocidad máxima y la posición de la PTU
    // 2. Habilitar o deshabilitar el movimiento del robot
    // 3. Obtener la pose del robot
    Subscriber_initialize = this -> create_subscription<std_msgs::msg::String>("/initialize_hunter_params",10,std::bind(&hunter_motion::initialize,this,_1));
    Subscriber_start_stop = this -> create_subscription<std_msgs::msg::Bool>("/start_stop_hunter",10,std::bind(&hunter_motion::enable_function,this,_1)); 
    Subscriber_pose = this -> create_subscription<sensor_msgs::msg::NavSatFix>("/hunter/fix",10,std::bind(&hunter_motion::process_pose,this,_1));

    max_vel = 0.0;
    enable = false;

    // Este es el parámetro que va a definir cuanto se va a acercar el robot al punto marcado en la trayectoria antes de pasar al siguiente,
    // por lo que va a controlar la forma de la trayectoria reprodcida por el robot
    lookahead = 0.5;

    primera_vez_brazo = true;
    inicializacion_brazo = true;
}

hunter_motion::~hunter_motion()
{
    RCLCPP_ERROR(this->get_logger(), "SALIENDO DE LA EJECUCIÓN");
}

// ESTA ES LA FUNCIÓN DE CALLBACK ENCARGADA DE PROCESAR LA POSE DEL ROBOT
void hunter_motion::process_pose(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    // La pose del robot se va a obtener solo cuando se haya habilitado el movimiento del robot
    if (enable==true)
    {
        float dist;

        // Del mensaje que se recibe por el topic del GPS, se obtiene la latitud y longitud del robot
        double latitude_hunter = msg->latitude;
        double longitude_hunter = msg->longitude;

        // Esta función se encarga de pasar de coordenadas geográficas a coordenadas cartesianas en el sistema UTM
        GeographicLib::UTMUPS::Forward(latitude_hunter, longitude_hunter, zone, northp, x_hunter, y_hunter);

        // Se referencia la posción del robot con respecto a la posición de la PTU construyendo el sistema de referencia local
        x_hunter=x_hunter-x_ptu;
        y_hunter=y_hunter-y_ptu;

        // Se define de forma estática (se mantiene entre llamadas a la función) la posición anterior del robot, para el cálculo de la orientación
        static double last_x_hunter = x_hunter;
        static double last_y_hunter = y_hunter;

        // Se calcula la distancia entre la posición actual y la anterior del robot
        dist = sqrt ((x_hunter-last_x_hunter)*(x_hunter-last_x_hunter)+(y_hunter-last_y_hunter)*(y_hunter-last_y_hunter));

        // Para evitar calculos de la orientación incorrecta, se verifica que la distancia entre posiciones del robot sea mayor a un valor mínimo
        if (dist>0.5)
        {
            // Se calcula la orientación del robot con respecto al eje x y en un rango de 0 a 2*pi
            theta_hunter =atan2((y_hunter-last_y_hunter),(x_hunter-last_x_hunter));
            if (theta_hunter<0)
            {
                theta_hunter = theta_hunter + 2*M_PI;
            }

            // Se actualiza la posición anterior del robot
            last_x_hunter = x_hunter;
            last_y_hunter = y_hunter;
        }
    }

}

// ESTA ES LA FUNCIÓN DE CALLBACK ENCARGADA DE HABILITAR O DESHABILITAR EL MOVIMIENTO DEL ROBOT
void hunter_motion::enable_function(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (enable)
    {
        enable = false;
    }
    else
    {
        enable = true;
    }
    RCLCPP_INFO(this->get_logger(),"RECIBIDO %i", enable);
}

// ESTA ES LA FUNCIÓN DE CALLBACK ENCARGADA DE INICIALIZAR LOS PARÁMETROS DEL ROBOT QUE SON:
// VELOCIDAD MÁXIMA, PUNTO DE REFERENCIA DE LA PTU Y TRAYECTORIA A SEGUIR
void hunter_motion::initialize(const std_msgs::msg::String::SharedPtr msg)
{
    // Se vacía el array para que entre llamadas consecutivas esto funcione igual que la primera vez
    path_points.clear();

    // Se publica la pose de inicio del brazo para inicializar el sistema
    if (inicializacion_brazo)
    {
        inicializacion_brazo = false;
        geometry_msgs::msg::Pose Pose_inicio;
        Pose_inicio.position.x = 0.2;
        Pose_inicio.position.y = 0.0;
        Pose_inicio.position.z = 1.0;
        Pose_inicio.orientation.x = 0.0;
        Pose_inicio.orientation.y = -0.7071;
        Pose_inicio.orientation.z = 0.0;
        Pose_inicio.orientation.w = 0.7071;
        Publisher_arm_pose->publish(Pose_inicio);
    }
    

    // Dado que los parámetros del robot se reciben en formato JSON, se procesan cada uno de los campos
    json json_msg = json::parse(msg->data);
    max_vel = json_msg["vel"];
    RCLCPP_INFO(this->get_logger(),"La velocidad es: %f",max_vel);

    double latitude_ptu = json_msg["point_ptu"]["latitude_ptu"];
    double longitude_ptu = json_msg["point_ptu"]["longitude_ptu"];
    GeographicLib::UTMUPS::Forward(latitude_ptu, longitude_ptu, zone, northp, x_ptu, y_ptu);

    // Se obtienen los puntos de la trayectoria a seguir y se convierten a coordenadas cartesianas UTM en primer lugar y,
    // posteriormente se referencia con respecto a la PTU
    for (const auto& point : json_msg["points"])
    {
        double latitude=point["latitude"];
        double longitude=point["longitude"];
        double x,y;
        GeographicLib::UTMUPS::Forward(latitude,longitude,zone, northp, x, y);
        x = x - x_ptu;
        y = y - y_ptu;
        path_points.push_back({x,y});
    }
    
    for (const auto& p : path_points) {
        RCLCPP_INFO(this->get_logger(), "Punto: x = %.9f, y = %.9f", p.first, p.second);
    }
}

// ESTA FUNCIÓN SE ENCARGA DE CONTROLAR AL ROBOT PARA GENERAR LA TRAYECTORIA QUE DEBE DE SEGUIR
/**
 * @brief Controla el seguimiento de la trayectoria del robot hunter.
 *
 * Esta función es responsable de mover el robot a lo largo de una trayectoria predefinida, siguiendo secuencialmente
 * los puntos de referencia almacenados en el vector `path_points`. Utiliza un controlador proporcional para calcular
 * las velocidades lineal y angular necesarias para alcanzar cada punto. La función mantiene un índice estático para
 * recordar el punto objetivo actual entre llamadas. Cuando el robot está dentro de una distancia determinada por
 * `lookahead` respecto al punto actual, avanza al siguiente. Si se han alcanzado todos los puntos, publica un mensaje
 * de finalización y detiene el movimiento.
 *
 * La función solo ejecuta el seguimiento de trayectoria si la bandera `enable` está en true. Si está deshabilitada,
 * ordena al robot detenerse. Los comandos de velocidad se publican mediante `Publisher_vel`, y la finalización se
 * notifica a través de `Publisher_end_hunter`.
 *
 * Supone que las siguientes variables miembro están definidas y actualizadas en otras partes:
 * - path_points: std::vector<std::pair<float, float>> que contiene los puntos de la trayectoria.
 * - x_hunter, y_hunter: Posición actual del robot.
 * - theta_hunter: Orientación actual del robot (en radianes).
 * - max_vel: Velocidad lineal máxima.
 * - lookahead: Umbral de distancia para cambiar al siguiente punto.
 * - enable: Bandera booleana para habilitar/deshabilitar el seguimiento de trayectoria.
 * - Publisher_vel: Publicador de comandos de velocidad.
 * - Publisher_end_hunter: Publicador para notificar la finalización de la trayectoria.
 */
void hunter_motion::hunter_trajectory()
{
    // Se define de forma estática (se mantiene entre llamadas a la función) el índice que apunta al punto de la trayectoria que se debe de seguir
    static long unsigned int index = 0;
    geometry_msgs::msg::Twist action;
    if(enable==true)
    {
        //arm_orientation();
        // Se comprueba que el índice no se haya salido del rango de puntos de la trayectoria
        if (index >= path_points.size())
        {
            std_msgs::msg::Bool fin;
            fin.data = true;
            Publisher_end_hunter -> publish(fin);

            return;
        }

        //RCLCPP_INFO(this->get_logger(),"BUSCANDO PUNTO: %lu",index);

        // Se obtiene el punto de la trayectoria que se debe de seguir
        float x_dest = path_points[index].first;
        float y_dest = path_points[index].second;

        // Se calcula la distancia y el ángulo que se debe de seguir para llegar al punto de la trayectoria obteniendo el ángulo en el intervalo 0 a 2*pi
        float dist = sqrt((x_dest-x_hunter)*(x_dest-x_hunter)+(y_dest-y_hunter)*(y_dest-y_hunter));
        float desired_theta = atan2((y_dest-y_hunter),(x_dest-x_hunter));

        if (desired_theta<0)
        {
            desired_theta = desired_theta + 2*M_PI;
        }

        // Se calcula el error entre la orientación deseada y la actual del robot, con lo que se calcula la curvatura que se debe de seguir haciendo uso
        // de un controlador proporcional
        float error = desired_theta - theta_hunter;
        float curvature = 2 * sin(error) / dist;

        // Se calcula la acción de control que se debe de ejecutar para que el robot siga la trayectoria
        action.linear.x = max_vel;
        action.angular.z = curvature*max_vel;

        // Se comprueba si la distancia entre el robot y el punto de la trayectoria es menor al valor establecido en lookahead
        // de ser así, se pasa al siguiente punto
        if(dist<lookahead)
        {
            index++;
        }
    }
    else
    {
        
        action.linear.x = 0.0;
        action.angular.z = 0.0;
    }
    Publisher_vel->publish(action);
}

// ESTA FUNCIÓN SE ENCARGA DE CALCULAR LA ORIENTACIÓN DEL BRAZO PARA QUE LA NORMAL DEL REFLECTOR SEA PARALELA AL EJE ÓPTICO DE LA CÁMARA
void hunter_motion::arm_orientation()
{
    if(enable==true)
    {
        double dx = -x_hunter;
        double dy = -y_hunter;

        double yaw = atan2(dy, dx);

        tf2::Quaternion q_base(0.0, -0.7071, 0.0, 0.7071);
        tf2::Quaternion q_yaw;
        q_yaw.setRPY(0.0, 0.0, yaw);

        tf2::Quaternion q_final = q_yaw * q_base;
        static tf2::Quaternion q_ultima_;
        q_final.normalize();

        double diferencia_ang = q_ultima_.angleShortestPath(q_final);

        RCLCPP_INFO(this->get_logger(),"El valor de la diferencia angular es: %f", diferencia_ang);

        if (primera_vez_brazo || diferencia_ang > 0.087)
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = 0.2;
            pose.position.y = 0.0;
            pose.position.z = 1.0;
            pose.orientation = tf2::toMsg(q_final);

            Publisher_arm_pose->publish(pose);
            q_ultima_ = q_final;
            primera_vez_brazo = false;
        }
    }
}

int main(int argc, char *argv[])
{ 
    rclcpp::init(argc, argv);
    auto node = std::make_shared<hunter_motion>();

    auto logger = node->get_logger();
    logger.set_level(rclcpp::Logger::Level::Info);

    rclcpp::Rate loop_rate(10.000);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
        node->hunter_trajectory();
    }

    rclcpp::shutdown();
    return 0;
}