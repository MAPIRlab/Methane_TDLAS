#include "interbotix_obstacles_pkg/scanner_ptu_2.hpp"
using std::placeholders::_1;

// Constructor de la clase scanner_ptu_2
scanner_ptu_2::scanner_ptu_2() : Node("scanner_ptu_2")
{
    // Suscripción al tópico para iniciar el escáner
    Subscriptor_start = this->create_subscription<std_msgs::msg::Bool>("/Start_Scanner", 10, std::bind(&scanner_ptu_2::start_scanner,this, _1));
    // Suscripción al estado de la PTU (JointState)
    Subscriptor_Info = this->create_subscription<sensor_msgs::msg::JointState>("/PTU_Methane/joint_states",10, std::bind(&scanner_ptu_2::callback_estado,this, _1));
    // Publicador de comandos de velocidad para la PTU
    Publicador_vel = this->create_publisher<interbotix_xs_msgs::msg::JointSingleCommand>("/PTU_Methane/commands/joint_single",10);
    // Publicador para indicar el fin del escaneo
    Publicador_fin = this->create_publisher<std_msgs::msg::Bool>("/Fin_PTU",10);

    // Cliente para cambiar el modo de operación de la PTU
    client_ = this->create_client<interbotix_xs_msgs::srv::OperatingModes>("/PTU_Methane/set_operating_modes");
    position_pan_ptu = 0.0;
    escaneo = false;
    recuperacion = false;
    angulo_inicio = 0.0; // Ángulo de inicio (debe completarse con la posición de partida)
    angulo_fin = -3.14;  // Ángulo final (debe completarse con la posición de fin)
    inicializacion_pan = true;
    inicializacion_tilt = false;
    inicializacion = true;
}

// Destructor de la clase
scanner_ptu_2::~scanner_ptu_2()
{
    RCLCPP_ERROR(this->get_logger(),"FIN DE LA EJECUCIÓN");
}

// Bucle principal que controla el movimiento de la PTU
void scanner_ptu_2::main_loop()
{
    float vel_escaneo = -0.17;      // Velocidad de escaneo
    float vel_recuperacion = 0.17;  // Velocidad de recuperación
    float vel_reposo = 0.0;         // Velocidad de reposo
    interbotix_xs_msgs::msg::JointSingleCommand accion;
    std_msgs::msg::Bool fin;
    fin.data=false;

    accion.name = "pan";
    accion.cmd = vel_reposo;
    if(escaneo)
    {
        // Controla los ángulos para evitar que la PTU se pase del límite
        accion.cmd = vel_escaneo;
        if(position_pan_ptu<angulo_fin) // Si se alcanza el ángulo final, termina el escaneo
        {
            RCLCPP_INFO(this->get_logger(),"FIN DEL ESCANEO, RECUPERANDO");
            escaneo = false;
            recuperacion = true;
            fin.data = true;
            //Publicador_fin->publish(fin); // Publica que terminó el escaneo
        }
    }

    if(recuperacion)
    {
        // Recupera la posición inicial de la PTU
        accion.cmd = vel_recuperacion;
        if (position_pan_ptu>-10*M_PI/180)
        {
            accion.cmd = 0.17;
        }
        if(position_pan_ptu>angulo_inicio)
        {
            accion.cmd = 0.0;
            recuperacion = false;
            escaneo = true;
            RCLCPP_INFO(this->get_logger(),"FIN DE LA RECUPERACION");
        }
    }
    Publicador_vel->publish(accion);
}

// Callback para iniciar o detener el escaneo según el mensaje recibido
void scanner_ptu_2::start_scanner(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        escaneo=true;
    }
    else
    {
        escaneo=false;
    }
}

// Callback que recibe el estado de la PTU y gestiona la inicialización y control de posición
void scanner_ptu_2::callback_estado(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    position_pan_ptu = msg->position[0];
    position_tilt_ptu = msg->position[1];
    interbotix_xs_msgs::msg::JointSingleCommand accion;
    std_msgs::msg::Bool fin;

    static double last_position_pan_ptu = position_pan_ptu;

    // Corrige saltos bruscos en la posición del pan
    if (abs(last_position_pan_ptu-position_pan_ptu)>0.3 and (position_pan_ptu < -3.14 and position_pan_ptu > -3.15))
    {
        position_pan_ptu = last_position_pan_ptu;
    }
    else
    {
        last_position_pan_ptu = position_pan_ptu;
    }

    // Inicialización de la PTU (llevar pan y tilt a cero)
    if (inicializacion)
    {
        if(inicializacion_pan)
        {
            if (position_pan_ptu<0.0)
            {
                accion.name = "pan";
                accion.cmd = 0.1;
            }
            else
            {
                accion.name = "pan";
                accion.cmd = -0.1;
            }
            Publicador_vel->publish(accion);  
        }

        // Si el pan está cerca de cero, pasa a inicializar el tilt
        if(position_pan_ptu>-0.1 and position_pan_ptu<0.1)
        {
            inicializacion_pan = false;
            inicializacion_tilt = true;
        }

        if (inicializacion_tilt)
        {
            if (position_tilt_ptu<0.0)
            {
                accion.name = "tilt";
                accion.cmd = 0.1;

                if (position_tilt_ptu>-0.3)
                {
                    accion.cmd = 0.03;
                }
            }
            else
            {
                accion.name = "tilt";
                accion.cmd = -0.1;

                if (position_tilt_ptu<0.3)
                {
                    accion.cmd = -0.03;
                }
            }
            Publicador_vel->publish(accion);  
        }

        // Si el tilt está cerca de cero, termina la inicialización
        if(position_tilt_ptu>-0.01 and position_tilt_ptu<0.01)
        {
            inicializacion_pan = false;
            inicializacion_tilt = false;
            inicializacion = false;
            accion.name = "tilt";
            fin.data = true;
            //Publicador_fin->publish(fin);
            accion.cmd = 0.0;
            Publicador_vel->publish(accion); 
            RCLCPP_INFO(this->get_logger(),"INICIALIZADO");
            Publicador_vel->publish(accion); 
        }
    }
    

}

// Función principal del nodo
int main(int argc, char *argv[])
{
    // Inicializa ROS2 y el nodo
    rclcpp::init(argc, argv);
    auto node = std::make_shared<scanner_ptu_2>();

    auto logger = node->get_logger();
    logger.set_level(rclcpp::Logger::Level::Info);

    // Bucle principal a 10 Hz (frecuencia de los mensajes de la PTU)
    rclcpp::Rate loop_rate(10.000);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        node->main_loop();
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}