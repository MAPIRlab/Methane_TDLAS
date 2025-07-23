#include "interbotix_obstacles_pkg/scanner_ptu_3D.hpp"
using std::placeholders::_1;

// Constructor de la clase scanner_ptu_3D
scanner_ptu_3D::scanner_ptu_3D() : Node("scanner_ptu_3D")
{
    // Suscripción al tópico para iniciar el escaneo
    Subscriptor_start = this->create_subscription<std_msgs::msg::Bool>("/Start_Scanner", 10, std::bind(&scanner_ptu_3D::start_scanner,this, _1));
    // Suscripción al estado de la PTU (JointState)
    Subscriptor_Info = this->create_subscription<sensor_msgs::msg::JointState>("/PTU_Methane/joint_states",10, std::bind(&scanner_ptu_3D::callback_estado,this, _1));
    // Publicador de comandos de velocidad para la PTU
    Publicador_vel = this->create_publisher<interbotix_xs_msgs::msg::JointSingleCommand>("/PTU_Methane/commands/joint_single",10);
    // Publicador para indicar el fin del escaneo
    Publicador_fin = this->create_publisher<std_msgs::msg::Bool>("/Fin_PTU",10);

    // Cliente para cambiar el modo de operación de la PTU
    client_ = this->create_client<interbotix_xs_msgs::srv::OperatingModes>("/PTU_Methane/set_operating_modes");
    // Inicialización de variables de estado
    position_pan_ptu = 0.0;
    escaneo = false;
    recuperacion = false;
    angulo_inicio = 0.0; // Ángulo inicial (debe completarse)
    angulo_fin = -3.14; // Ángulo final (debe completarse)
    inicializacion_pan = true;
    inicializacion_tilt = false;
    inicializacion = true;
    tramo1 = true;
    tramo2 = false;
    tramo3 = false;
    tramo4 = false;
    tramo5 = false;
    tramo6 = false;
    tramo7 = false;
}

// Destructor de la clase
scanner_ptu_3D::~scanner_ptu_3D()
{
    RCLCPP_ERROR(this->get_logger(),"FIN DE LA EJECUCIÓN");
}

// Bucle principal del nodo
void scanner_ptu_3D::main_loop()
{
    float vel_escaneo = -0.025;
    float vel_recuperacion = 0.3;
    float vel_reposo = 0.0;
    interbotix_xs_msgs::msg::JointSingleCommand accion;
    std_msgs::msg::Bool fin;
    fin.data=false;

    accion.name = "pan";
    accion.cmd = vel_reposo;
    if(escaneo)
    {
        // Controla la secuencia de escaneo y los ángulos de la PTU
        secuencia_escaneo();
        // Si todos los tramos han terminado, se finaliza el escaneo
        if(tramo1==false and tramo2==false and tramo3==false and tramo4==false and tramo5==false and tramo6==false and tramo7==false)
        {
            RCLCPP_INFO(this->get_logger(),"FIN DEL ESCANEO, RECUPERANDO");
            escaneo = false;
            recuperacion = true;
            fin.data = true;
            Publicador_fin->publish(fin); // Publica que terminó el escaneo
        }
    }

    if(recuperacion)
    {
        // Recupera la posición inicial de la PTU tras el escaneo
        accion.cmd = vel_recuperacion;
        if (position_pan_ptu>-10*M_PI/180)
        {
            accion.cmd = 0.1;
        }
        if(position_pan_ptu>angulo_inicio)
        {
            accion.cmd = 0.0;
            recuperacion = false;
            RCLCPP_INFO(this->get_logger(),"FIN DE LA RECUPERACION");
        }
    }
    // Publica el comando de velocidad calculado
    Publicador_vel->publish(accion);
}

// Secuencia de movimientos para el escaneo 3D
void scanner_ptu_3D::secuencia_escaneo()
{
    float vel_escaneo = 0.025;
    float vel_recuperacion = 0.3;
    float vel_reposo = 0.0;
    interbotix_xs_msgs::msg::JointSingleCommand accion;
    // Cada tramo representa un movimiento específico de pan o tilt
    if (tramo1)
    {
        accion.name = "tilt";
        accion.cmd = vel_escaneo;

        if(position_tilt_ptu>0.17453)
        {
            tramo1=false;
            tramo2=true;
            accion.name = "tilt";
            accion.cmd = vel_reposo;
            Publicador_vel->publish(accion);
            return; 
        }
    }
    if (tramo2)
    {
        accion.name = "pan";
        accion.cmd = -vel_escaneo;

        if(position_pan_ptu<-3.14)
        {
            tramo2=false;
            tramo3=true;
            accion.name = "pan";
            accion.cmd = vel_reposo;
            Publicador_vel->publish(accion); 
            return;
        }
    }
    if (tramo3)
    {
        accion.name = "tilt";
        accion.cmd = -vel_escaneo;

        if(position_tilt_ptu<0.0)
        {
            tramo3=false;
            tramo4=true;
            accion.name = "tilt";
            accion.cmd = vel_reposo;
            Publicador_vel->publish(accion); 
            return;
        }
    }
    if (tramo4)
    {
        accion.name = "pan";
        accion.cmd = vel_escaneo;

        if(position_pan_ptu>0)
        {
            tramo4=false;
            tramo5=true;
            accion.name = "pan";
            accion.cmd = vel_reposo;
            Publicador_vel->publish(accion); 
            return;
        }
    }
    if(tramo5)
    {
        accion.name = "tilt";
        accion.cmd = -vel_escaneo;

        if(position_tilt_ptu<-0.17453)
        {
            tramo5=false;
            tramo6=true;
            accion.name = "tilt";
            accion.cmd = vel_reposo;
            Publicador_vel->publish(accion); 
            return;
        }
    }
    if(tramo6)
    {
        accion.name = "pan";
        accion.cmd = -vel_escaneo;

        if(position_pan_ptu<-3.14)
        {
            tramo6=false;
            tramo7=true;
            accion.name = "pan";
            accion.cmd = vel_reposo;
            Publicador_vel->publish(accion); 
            return;
        }
    }
    if(tramo7)
    {
        accion.name = "tilt";
        accion.cmd = vel_escaneo;

        if(position_tilt_ptu>0)
        {
            tramo7=false;
            accion.name = "tilt";
            accion.cmd = vel_reposo;
            Publicador_vel->publish(accion); 
            return;
        }
    }

    // Publica el comando de movimiento correspondiente al tramo actual
    Publicador_vel->publish(accion);  
}

// Callback para iniciar o detener el escaneo
void scanner_ptu_3D::start_scanner(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        escaneo=true;
        RCLCPP_INFO(this->get_logger(),"INICIO ESCANEO");
    }
    else
    {
        escaneo=false;
    }
}

// Callback para actualizar el estado de la PTU y controlar la inicialización
void scanner_ptu_3D::callback_estado(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    position_pan_ptu = msg->position[0];
    position_tilt_ptu = msg->position[1];
    interbotix_xs_msgs::msg::JointSingleCommand accion;
    std_msgs::msg::Bool fin;

    static double last_position_pan_ptu = position_pan_ptu;
    static double last_position_tilt_ptu = position_tilt_ptu;

    // Filtro para evitar saltos bruscos en la posición del pan
    if (abs(last_position_pan_ptu-position_pan_ptu)>0.3 and (position_pan_ptu < -3.14 and position_pan_ptu > -3.15))
    {
        position_pan_ptu = last_position_pan_ptu;
    }
    else
    {
        last_position_pan_ptu = position_pan_ptu;
    }

    // Filtro para evitar saltos bruscos en la posición del tilt
    if (abs(last_position_tilt_ptu-position_tilt_ptu)>0.3 and (position_tilt_ptu < -3.14 and position_tilt_ptu > -3.15))
    {
        position_tilt_ptu = last_position_tilt_ptu;
    }
    else
    {
        last_position_tilt_ptu = position_tilt_ptu;
    }

    // Proceso de inicialización: centra la PTU en pan y tilt
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

        if(position_tilt_ptu>-0.01 and position_tilt_ptu<0.01)
        {
            inicializacion_pan = false;
            inicializacion_tilt = false;
            inicializacion = false;
            accion.name = "tilt";
            fin.data = true;
            Publicador_fin->publish(fin);
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
    auto node = std::make_shared<scanner_ptu_3D>();

    auto logger = node->get_logger();
    logger.set_level(rclcpp::Logger::Level::Info);

    // Bucle principal a 10 Hz (frecuencia de la PTU)
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