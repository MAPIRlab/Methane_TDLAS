#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "interbotix_xs_msgs/msg/joint_group_command.hpp"
#include "interbotix_xs_msgs/msg/joint_single_command.hpp"
#include "interbotix_xs_msgs/msg/joint_trajectory_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nlohmann/json.hpp"
#include <eigen3/Eigen/Dense>
#include <GeographicLib/UTMUPS.hpp>
#include <cmath>

class mov_PTU : public rclcpp::Node
{
    public:
    //CONSTRUCOTR DE LA CLASE
    mov_PTU();
    //DESTRUCTOR DE LA CLASE
    ~mov_PTU();
    //FUNCIÓN CON LA QUE SE VAN A EXTRAER LOS MENSAJES QUE LLEGAN POR EL TOPIC QUE PUBLICA EL NODO QUE DETECTA EL PATRÓN
    void deserializar(const std_msgs::msg::String::SharedPtr msg);
    //FUNCIÓN PRIMCIPAL ENCARGADA DE COMANDAR LA PTU
    void mover_PTU();
    //FUNCIÓN DE CALLBACK DEL TOPIC QUE PUBLICA LA PROPIA PTU, EN LA QUE VEMOS LAS POSICIONES Y LAS VELOCIDADES DE ESTA
    void callback_estado(const sensor_msgs::msg::JointState::SharedPtr msg);
    //FUNCIÓN PARA CALCULAR EL VALOR DE LA VELOCIDAD QUE DEBEMOS DE DAR A LA PTU EN FUNCIÓN DE QUÉ MOTOR SEA Y DEL ERROR QUE SE TENGA
    float calcularPID(float error, float &error_prev, float &integral, float Kp, float Ki, float Kd, float dt);
    //FUNCIÓN PARA LLEVAR A CABO EL PID ADAPTATIVO
    void ajustar_PID();
    //FUNCIÓN PARA IMPLEMENTAR EL FILTRO DE KALMAN, PERMITIENDO HACER PREDICCIONES
    void aplicarFiltroKalman();
    //FUNCIÓN DE CALLBACK PARA EL TEMPORIZADOR DE PÉRDIDA DE MENSAJES
    void callback_recepcion();
    //FUNCIÓN DE CALLBACK PARA EL TOPIC DE LA SUBSCRIPCION A LA POSICION DEL HUNTER
    void callback_hunter_position(const std_msgs::msg::String::SharedPtr msg);
    //FUNCIÓN DE CALLBACK PARA EL TOPIC DE LA SUBSCRIPCION A LA POSICION DE LA PTU
    void callback_ptu_position(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    //VARIABLE PARA CALCULAR EL INCREMENTO DE TIEMPO ENTRE EJECUCIONES DEL CÓDIGO
    rclcpp::Time tiempo_anterior;
    //TEMPORIZADOR PARA DETECTAR PÉRDIDAS DE MENSAJES
    rclcpp::TimerBase::SharedPtr temporizador_recepcion;
    //VARIABLE QUE CONTABILIZA EL TIEMPO TRASCURRIDO ENTRE DOS EJECUCUIONES DEL BUCLE
    float dt;

    private:
    //SUBSCRIPTOR DEL TOPIC QUE PUBLICA EL NODO CAPAZ DE DETECTAR EL PATRÓN
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Subscriptor;
    //SUBSCRIPTOR AL TOPIC QUE PUBLICA LA INFORMACIÓN DE LA PTU 
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr Subscriptor_Info;
    //SUBSCRIPTOR AL GPS DE LA PTU
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr Subscriptor_pose_ptu;
    //SUBSCRIPTOR AL GPS DEL HUNTER
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Subscriptor_pose_hunter;
    //PUBLICADOR PARA COMANDAR LA PTU
    rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr Publicador;
    //PUBLICADOR DISTANCIA A LA QUE SE ENCUENTRA EL PATRÓN
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr Publicador_distancia;

    //VARIABLES QUE LLEGAN EN EL MENSAJE JSON
    float error_x, error_y, centro_imagen_x, centro_imagen_y;
    //VARIABLES DONDE SE GUARDA LA EQUIVALENTE RADIANES A PÍXELES
    float eq_x, eq_y;

    //VARIABLES PARA LLEVAR A CABO EL MODO DE SEGURIDAD DE LA PTU PARA EVITAR QUE SE PUEDA ROMPER
    bool limite_antihorario, limite_horario, limite_arriba, limite_abajo, aviso_1, aviso_2;

    //PARÁMETROS QUE SE VAN A USAR PARA CALCULAR LA VELOCIDAD A LA QUE SE DEBE MOVER LA PTU
    float error_prev_x, integral_x, error_prev_y, integral_y, ajuste_Kp, ajuste_Ki, v_max, v_min;

    //GANANCIAS DEL CONTROLADOR QUE SE IMPLEMENTA
    float Kp_x, Kp_y, Ki_x, Ki_y, Kd_x, Kd_y, Kp_x_4111, Kp_y_4111, Ki_x_4111, Ki_y_4111, Kd_x_4111, Kd_y_4111,  Kp_x_8911, Kp_y_8911, Ki_x_8911, Ki_y_8911, Kd_x_8911, Kd_y_8911;

    //PARAMETROS DE LAS RECTAS DE CALIBRACION DE LA CAMARA Y EL TDLAS
    float m_x, n_x, m_y, n_y, distancia;

    //VALORES DE LATITUD Y LONGITUD DE PTU Y HUNTER PARA CALCULAR LA DISTANCIA
    float latitude_hunter, latitude_ptu, longitude_hunter, longitude_ptu, current_pan, current_tilt;

    //MATRICES Y VECTORES NECESARIOS PARA LA APLICACIÓN DEL FILTRO DE KALMAN
    Eigen::Vector2f estado_x, estado_y, estado_pred_x, estado_pred_y;
    Eigen::Matrix2f A,P,Q,R,K;
    Eigen::Matrix<float,2,1> B;

    double a0, a1, a2, a3, a4, a5;
    float pixel_size;

    int encoder_zoom;
};