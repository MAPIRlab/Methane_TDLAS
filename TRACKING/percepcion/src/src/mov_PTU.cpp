#include "percepcion/mov_PTU.hpp"
using std::placeholders::_1;
using json = nlohmann::json;


// SE IMPLEMENTA EL CONSTRUCTOR DE LA CLASE
/**
 * @brief Constructor for the mov_PTU class.
 *
 * Initializes the ROS2 node with the name "mov_PTU" and sets up all required publishers, subscribers, and parameters.
 * 
 * - Creates subscriptions to:
 *   - "/Info_Posicion" (std_msgs::msg::String): Receives position information.
 *   - "/PTU_Methane/joint_states" (sensor_msgs::msg::JointState): Receives PTU joint states.
 *   - "/hunter_position" (std_msgs::msg::String): Receives hunter position information.
 *   - "/fix" (sensor_msgs::msg::NavSatFix): Receives GPS position for PTU.
 * - Creates publishers for:
 *   - "/PTU_Methane/commands/joint_group" (interbotix_xs_msgs::msg::JointGroupCommand): Publishes joint group commands.
 *   - "/Distancia" (std_msgs::msg::Float32): Publishes distance information.
 * - Declares and retrieves various parameters for PID control, limits, calibration, zoom, sensor, and adaptive PID adjustment.
 * - Initializes Kalman filter matrices (A, B, P, Q, R) and state vectors (estado_x, estado_y, estado_pred_x, estado_pred_y).
 * - Initializes variables for image center, PID control, and status flags.
 * - Sets up a wall timer for message reception loss detection.
 * - Captures the initial time for loop execution timing.
 */
mov_PTU::mov_PTU() : Node("mov_PTU")
{
    // Se crean los dos subscriptores y el publicador
    Subscriptor = this -> create_subscription<std_msgs::msg::String>("/Info_Posicion", 10, std::bind(&mov_PTU::deserializar, this, _1));
    Publicador = this -> create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>("/PTU_Methane/commands/joint_group", 10);
    Publicador_distancia = this -> create_publisher<std_msgs::msg::Float32>("/Distancia",10);
    Subscriptor_Info = this -> create_subscription<sensor_msgs::msg::JointState>("/PTU_Methane/joint_states", 10, std::bind(&mov_PTU::callback_estado,this,_1));
    Subscriptor_pose_hunter = this -> create_subscription<std_msgs::msg::String>("/hunter_position", 10, std::bind(&mov_PTU::callback_hunter_position,this,_1));
    Subscriptor_pose_ptu = this -> create_subscription<sensor_msgs::msg::NavSatFix>("/fix",10, std::bind(&mov_PTU::callback_ptu_position,this,_1));

    // Parámetros PID (ajustables), su ajuste se ha hecho empíricamente
    this->declare_parameter<double>("pid.Kp_x_4111", 0.0);
    this->declare_parameter<double>("pid.Ki_x_4111", 0.0);
    this->declare_parameter<double>("pid.Kd_x_4111", 0.0);
    this->declare_parameter<double>("pid.Kp_y_4111", 0.0);
    this->declare_parameter<double>("pid.Ki_y_4111", 0.0);
    this->declare_parameter<double>("pid.Kd_y_4111", 0.0);

    this->declare_parameter<double>("pid.Kp_x_8911", 0.0);
    this->declare_parameter<double>("pid.Ki_x_8911", 0.0);
    this->declare_parameter<double>("pid.Kd_x_8911", 0.0);
    this->declare_parameter<double>("pid.Kp_y_8911", 0.0);
    this->declare_parameter<double>("pid.Ki_y_8911", 0.0);
    this->declare_parameter<double>("pid.Kd_y_8911", 0.0);


    this->declare_parameter<double>("limits.v_max", 0.0);
    this->declare_parameter<double>("limits.v_min", 0.0);


    this->declare_parameter<double>("calib.m_x", 0.0);
    this->declare_parameter<double>("calib.n_x", 0.0);
    this->declare_parameter<double>("calib.m_y", 0.0);
    this->declare_parameter<double>("calib.n_y", 0.0);


    this->declare_parameter<double>("zoom.a5", 0.0);
    this->declare_parameter<double>("zoom.a4", 0.0);
    this->declare_parameter<double>("zoom.a3", 0.0);
    this->declare_parameter<double>("zoom.a2", 0.0);
    this->declare_parameter<double>("zoom.a1", 0.0);
    this->declare_parameter<double>("zoom.a0", 0.0);


    this->declare_parameter<double>("sensor.pixel_size", 0.0);


    this->declare_parameter<double>("pid_adapt.ajuste_Kp", 0.0);
    this->declare_parameter<double>("pid_adapt.ajuste_Ki", 0.0);


    this->get_parameter("pid.Kp_x_4111", Kp_x_4111);
    this->get_parameter("pid.Ki_x_4111", Ki_x_4111);
    this->get_parameter("pid.Kd_x_4111", Kd_x_4111);
    this->get_parameter("pid.Kp_y_4111", Kp_y_4111);
    this->get_parameter("pid.Ki_y_4111", Ki_y_4111);
    this->get_parameter("pid.Kd_y_4111", Kd_y_4111);

    this->get_parameter("pid.Kp_x_4111", Kp_x_8911);
    this->get_parameter("pid.Ki_x_4111", Ki_x_8911);
    this->get_parameter("pid.Kd_x_4111", Kd_x_8911);
    this->get_parameter("pid.Kp_y_4111", Kp_y_8911);
    this->get_parameter("pid.Ki_y_4111", Ki_y_8911);
    this->get_parameter("pid.Kd_y_4111", Kd_y_8911);


    this->get_parameter("limits.v_max", v_max);
    this->get_parameter("limits.v_min", v_min);


    this->get_parameter("calib.m_x", m_x);
    this->get_parameter("calib.n_x", n_x);
    this->get_parameter("calib.m_y", m_y);
    this->get_parameter("calib.n_y", n_y);


    this->get_parameter("zoom.a5", a5);
    this->get_parameter("zoom.a4", a4);
    this->get_parameter("zoom.a3", a3);
    this->get_parameter("zoom.a2", a2);
    this->get_parameter("zoom.a1", a1);
    this->get_parameter("zoom.a0", a0);


    this->get_parameter("sensor.pixel_size", pixel_size);


    this->get_parameter("pid_adapt.ajuste_Kp", ajuste_Kp);
    this->get_parameter("pid_adapt.ajuste_Ki", ajuste_Ki);

    // Se crea el temporizador para la pérdida de mensajes
    temporizador_recepcion = this->create_wall_timer (std::chrono::milliseconds(500), std::bind(&mov_PTU::callback_recepcion, this));

    // Se coge el primer tiempo, para el cálculo del tiempo de ejecución del bucle
    tiempo_anterior=now();

    // Se definen las matrices de Kalman
    A << 1, 0.1,
         0, 1;
    B << 0, 1;
    P << 1, 0,
         0, 1;
    Q << 0.07, 0,
         0, 0.07;
    R << 0.003, 0,
         0, 0.003;
    estado_x << 0, 0;
    estado_y << 0, 0;
    estado_pred_x << 0, 0;
    estado_pred_y << 0, 0;

    // Se definen el resto de variables para el funcionamiento del nodo
    centro_imagen_x = 0.0;
    centro_imagen_y = 0.0;
    eq_x = 0.0;
    eq_y = 0.0;
    error_prev_x=0.0;
    error_prev_y=0.0;
    integral_x=0.0;
    integral_y=0.0;
    aviso_1=false;
    aviso_2=false;
}

// SE IMPLEMENTA EL DESTRUCTOR DE LA CLASE
mov_PTU::~mov_PTU()
{
    RCLCPP_ERROR(this->get_logger(), "SALIENDO DE LA EJECUCIÓN");
}

// SE IMPLEMENTA LA FUNCIÓN DE CALLBACK DE LA POSICIÓN GPS DEL HUNTER
void mov_PTU::callback_hunter_position(const std_msgs::msg::String::SharedPtr msg)
{
    std::string mensaje_json = msg->data;
    json mensaje = json::parse(mensaje_json);
    latitude_hunter = mensaje["Latitude"];
    longitude_hunter = mensaje["Longitude"];
}

void mov_PTU::callback_ptu_position(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    latitude_ptu = msg->latitude;
    longitude_ptu = msg->longitude;
}

// SE IMPLEMENTA LA FUNCIÓN DE CALLBACK DEL TEMPORIZADOR DE RECEPCIÓN DE MENSAJES
void mov_PTU::callback_recepcion()
{
    /*
     Esta función coloca el error tanto en el eje X como en el Y cuando detecta que lleva más de 0.5 segundos sin 
     recibir ningún mensaje de información de donde se encuentra el marcador, para evitar así que la PTU se mueva más de 
     lo necesario y lo pierda. Esto se debe a que la PTU se queda con la velocidad del último mensaje recibido
    */
    RCLCPP_INFO(this->get_logger(),"MARCADOR_PERDIDO");
    error_x=0.0;
    error_y=0.0;
}

// SE IMPLEMENTA LA FUNCIÓN DE CALLBACK USADA EN LA SUBSCRIPCIÓN AL TOPIC DEL ESTADO DE LA PTU
void mov_PTU::callback_estado(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    /*
     En esta función se trata de implementar la lógica necesario para que la PTU no se rompa al hacer un giro que no puede soportar
     esto es debido a que esta tiene un obstáculo físico sobre todo en el movimiento del 'pan' que es un cable, si se gira un ángulo
     mayor de lo esperado, este se puede romper.
     Para coger los sentidos horario, antihorario, arriba y abajo, se ha colocado la PTU con la salida de los cables orientada hacia nosotros
     mirándola desde esta perspectiva, se tiene horario y antihorario. Arriba se ha considerado con un cámara montada mirando hacia el lado contrario
     de la salida de los cables, al momento en el que la cámara queda con su objetivo mirando completamente hacia arriba. Abajo cuando ocurre
     lo contrario.
     Se han considerado unos límites lo suficientemente fiables para asegurar que un posible retraso en la recepción de mensajes por este topic
     no provoque la rotura de la PTU
    */
    if(msg->position[0] < M_PI/2 and msg->position[0] > -M_PI)
    {
        limite_antihorario=false;
        limite_horario=false;
        aviso_1=false;
    }
    else if(msg->position[0] <= -M_PI)
    {
        limite_antihorario=false;
        limite_horario=true;
        if(aviso_1==false)
        {
            RCLCPP_WARN(this->get_logger(),"MODO DE SEGURIDAD POR LIMITE EN SENTIDO HORARIO");
            aviso_1=true;
        }
    }
    else
    {
        limite_antihorario=true;
        limite_horario=false;
        if(aviso_1==false)
        {
            RCLCPP_WARN(this->get_logger(),"MODO DE SEGURIDAD POR LIMITE EN SENTIDO ANTIHORARIO");
            aviso_1=true;
        }
    }

    if(msg->position[1] < M_PI/2 and msg->position[1] > -M_PI/2)
    {
        limite_arriba=false;
        limite_abajo=false;
        aviso_2=false;
    }
    else if(msg->position[1] <= -M_PI/2)
    {
        limite_arriba=true;
        limite_abajo=false;
        if(aviso_2==false)
        {
            RCLCPP_WARN(this->get_logger(),"MODO DE SEGURIDAD POR LIMITE HACIA ARRIBA");
            aviso_2=true;
        }
    }
    else
    {
        limite_arriba=false;
        limite_abajo=true;
        if(aviso_2==false)
        {
            RCLCPP_WARN(this->get_logger(),"MODO DE SEGURIDAD POR LIMITE HACIA ABAJO");
            aviso_2=true;
        }
    }
}

// SE IMPLEMENTA LA FUNCIÓN DE CALLBACK DE RECEPCIÓN DE MENSAJES DEL NODO DE CAPTACIÓN DEL MARCADOR
void mov_PTU::deserializar(const std_msgs::msg::String::SharedPtr msg)
{
    /*
     El principal objetivo de esta función es el de extraer del mensaje en formato JSON que llega desde el nodo encargado del
     reconocimiento del patrón, los capos que se corresponden con las dimensiones en ancho y alto del patrón así como la posición en píxeles
     que ocupa el patrón en la imagen, para de esta forma calcular el centro de la imagen y con él, el error que se tiene en píxeles para ubicar el
     patrón en el centro de la imagen.
     En esta dfunción se resetea el temporizador de recepción de mensajes para evitar que se quede con la última consigna recibida.
    */

    temporizador_recepcion.reset();

    // Se calcula la desviación que hay entre la camara y el sensor a la distancia calculada EN CENTIMETROS
    float desviacion_x = (m_x * distancia + n_x)*10; 
    float desviacion_y = (m_y * distancia + n_y)*10;
    float distancia_mm = distancia * 1000;

    // Calculo el ángulo de desviación en el eje X e Y
    float theta_x = std::atan2(desviacion_x, distancia_mm);
    float theta_y = std::atan2(desviacion_y, distancia_mm);

    float FOVx, FOVy;
    std::string mensaje_json = msg->data;
    json mensaje = json::parse(mensaje_json);

    int width = mensaje["width"];
    int height = mensaje["height"];
    encoder_zoom = mensaje["encoder"];
    static int encoder_zoom_anterior = 0;

    if (encoder_zoom >= 8911 and encoder_zoom != encoder_zoom_anterior)
    {
        Kp_x = Kp_x_8911; Ki_x = Ki_x_8911; Kd_x = Kd_x_8911;
        Kp_y = Kp_y_8911; Ki_y = Ki_y_8911; Kd_y = Kd_y_8911;

        encoder_zoom_anterior = encoder_zoom;
    }
    else if (encoder_zoom >= 4111 and encoder_zoom != encoder_zoom_anterior)
    {
        Kp_x = Kp_x_4111; Ki_x = Ki_x_4111; Kd_x = Kd_x_4111;
        Kp_y = Kp_y_4111; Ki_y = Ki_y_4111; Kd_y = Kd_y_4111;

        encoder_zoom_anterior = encoder_zoom;
    }

    // Se calcula la distancia focal a la que corresponde el valor del encoder proporcionada
    float distancia_focal = a5*std::pow(encoder_zoom,5) + a4*std::pow(encoder_zoom,4) +a3*std::pow(encoder_zoom,3) + a2*std::pow(encoder_zoom,2) + a1*std::pow(encoder_zoom,1) + a0*std::pow(encoder_zoom,0);
    
    // Calculo el tamaño físico del sensor
    float S_w_mm = width * pixel_size;
    float S_h_mm = height * pixel_size;

    // Calculo el FOV en radianes tando en horizontal como en vertical 
    FOVx = 2.0 * std::atan2(S_w_mm/2.0, distancia_focal);
    FOVy = 2.0 * std::atan2(S_h_mm/2.0, distancia_focal);

    // Calculo la equivalencia de radianes por píxel

    eq_x = FOVx / width;
    //RCLCPP_INFO(this->get_logger(),"eq_X: %f", eq_x);
    eq_y = FOVy / height;

    // Calculo el desplazamiento en píxeles que supone la distancia propocionada
    float delta_x = theta_x / eq_x;
    float delta_y = theta_y / eq_y;

    centro_imagen_x = width / 2;
    centro_imagen_y = height / 2;

    int punto_equilibrio_x = static_cast<int>(std::round(centro_imagen_x + delta_x));
    int punto_equilibrio_y = static_cast<int>(std::round(centro_imagen_y + delta_y));

    //RCLCPP_INFO(this->get_logger(),"p_x: %i, p_y: %i", punto_equilibrio_x, punto_equilibrio_y);

    float pos_x = mensaje["pos_x"];
    float pos_y = mensaje["pos_y"];

    // Si esto no funciona, cambiar el punto_equilibrio por el centro_imagen para poder al menos ver si se queda cuadrado en el centro de la imagen
    error_x = pos_x - punto_equilibrio_x;
    error_y = pos_y - punto_equilibrio_y;
    // error_x = pos_x - centro_imagen_x;
    // error_y = pos_y - centro_imagen_y;
    temporizador_recepcion = this->create_wall_timer (std::chrono::milliseconds(500), std::bind(&mov_PTU::callback_recepcion, this));
}

// SE IMPLEMENTA LA FUNCIÓN PARA EL AJUSTE DE LAS GANANCIAS DEL PID (PID ADAPTATIVO)
void mov_PTU::ajustar_PID()
{
    /*
     En esta función lo que se hace es coger los valor de las ganancias proporcional e integral y se van ajustando para que el sistema cumpla con 
     las necesidades. Para ello se definen dos valores de ajuste que van a ser los valores que se va a modificar cada una de las ganancias. La lógica
     seguida es que cuando el error supere los 5 píxeles, se va incrementado el valor de la ganancia integral para que se corrija el error en régimen permanente
     El principal problema por el que se implementa el PID adaptativo es porque necesitamos un controlador que responda rápido y tenga poca sobreoscilación,
     tras el ajuste de los parámetros para la implementación de un PID estándar, se ha visto que para pequeños movimientos, funciona bien, el problema aparece 
     cuando se tiene un salto grande, que puede que a veces no converja. Lo que se hace es detectar las sobreoscilaciones y cuando estas ocurren, se empieza a bajar 
     el valor de la ganancia proporcional hasta que converge y se estabiliza. Una vez estabilizado se vuelve a colocar el mismo valor de ganancia proporcional que se
     tenía 
    */
    static int cruces_x = 0;
    static int cruces_y = 0;
    static int estabilidad_x = 0;
    static int estabilidad_y = 0;

    //AJUSTE DE ERROR EN REGIMEN PERMANENTE
    if (abs(error_x)>15)
    {
        Ki_x += ajuste_Ki;
    }
    else
    {
        if (Ki_x>0)
        {
            Ki_x -= ajuste_Ki;
        }
    }

    if (abs(error_y)>15)
    {
        Ki_y += ajuste_Ki;
    }
    else
    {
        if (Ki_y>0)
        {
            Ki_y -= ajuste_Ki;
        }
    }

    //AJUSTE SOBREOSCILACION
    if ((error_prev_x>0 and error_x<0) or (error_prev_x<0 and error_x>0))
    {
        cruces_x ++;
        estabilidad_x = 0;
    }
    else
    {
        estabilidad_x++;
    }

    if ((error_prev_y>0 and error_y<0) or (error_prev_y<0 and error_y>0))
    {
        cruces_y ++;
        estabilidad_y = 0;
    }
    else
    {
        estabilidad_y++;
    }

    if (cruces_x > 15)
    {
        if (Kp_x>0)
        {
            Kp_x -= 4*ajuste_Kp;
        }
        if (Ki_x > 0)
        {
            Ki_x -= ajuste_Ki;
        }
    }
    else if (cruces_x > 3)
    {

        if (Kp_x>0)
        {
            Kp_x -= 2*ajuste_Kp;
        }
        if (Ki_x > 0)
        {
            Ki_x -= ajuste_Ki;
        }
    }

    if (cruces_y > 15)
    {
        if (Kp_y>0)
        {
            Kp_y -= 4*ajuste_Kp;
        }
        if (Ki_y > 0)
        {
            Ki_y -= ajuste_Ki;
        }
    }
    else if (cruces_y > 3)
    {
        if (Kp_y>0)
        {
            Kp_y -= 2*ajuste_Kp;
        }
        if (Ki_y > 0)
        {
            Ki_y -= ajuste_Ki;
        }
    }

    if (estabilidad_x>10)
    {
        cruces_x=0;
        if (encoder_zoom == 8911)
        {
            Kp_x=Kp_x_8911;
        }
        else if (encoder_zoom == 4111)
        {
            Kp_x=Kp_x_4111;
        }
    }

    if (estabilidad_y>10)
    {
        cruces_y=0;
        if (encoder_zoom == 8911)
        {
            Kp_y=Kp_y_8911;
        }
        else if (encoder_zoom == 4111)
        {
            Kp_y=Kp_y_4111;
        }
    }

    //RCLCPP_INFO(this->get_logger(),"Kp_x: %f, Ki_x: %f, Kd_x: %f, cruces_x: %i",Kp_x,Ki_x,Kd_x,cruces_x);
    //RCLCPP_INFO(this->get_logger(),"Kp_y: %f, Ki_y: %f, Kd_y: %f, cruces_y: %i",Kp_y,Ki_y,Kd_y,cruces_y);
}

// SE IMPLEMENTA LA FUNCIÓN QUE PERMITE APLICAR LOS FILTROS DE KALMAN PARA HACER UN CONTROL PREDICTIVO
void mov_PTU::aplicarFiltroKalman()
{
    /*
     En esta función lo que se hace es aplicar la teoría y la matemática de los filtros de Kalman. Por lo cual lo que se hace lo primero es
     hacer una predicción del estado, calculando para ello la velocidad en función del error. Esto proporciona un estado predicho, el siguiente paso 
     es corregir esta predicción, para lo cual se incluye la información sensorial, se calcula la ganancia de Kalman y con esto se obtiene una medida del 
     estado muy cercana a la realidad. Una vez se tiene ese estado corregido, se hace una predicción en el tiempo, para lo cual se ha probado y se ha visto 
     el tiempo máximo de predicción admisible es de 0.2 segundos, ya que con una predicción más en el futuro, se hace que el sistema sea inestable
    */
    if(eq_x!=0.00000 and eq_y!=0.00000)
    {
        Eigen::Vector2f U_x(0, (calcularPID(error_x, error_prev_x, integral_x, Kp_x, Ki_x, Kd_x, dt)/eq_x));
        Eigen::Vector2f U_y(0, (calcularPID(error_y, error_prev_y, integral_y, Kp_y, Ki_y, Kd_y, dt)/eq_y));

        estado_x = A * estado_x + B * U_x(1);
        estado_y = A * estado_y + B * U_y(1);

        std::stringstream ss2;
        ss2 << "estado_x antes de corregirlo:\n";
        ss2 << estado_x(0)  << "\n";
        ss2 << estado_x(1)  << "\n";

        //RCLCPP_INFO(this->get_logger(), "%s", ss2.str().c_str());

        P = A * P * A.transpose() + Q;

        Eigen::Vector2f z_x, z_y;
        z_x << error_x, 0;
        z_y << error_y, 0;

        K = P * (P + R).inverse();

        estado_x = estado_x + K * (z_x - estado_x);
        estado_y = estado_y + K * (z_y - estado_y);

        std::stringstream ss;
        ss << "estado_x despues de corregirlo:\n";
        ss << estado_x(0)  << "\n";
        ss << estado_x(1)  << "\n";

        //RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

        estado_pred_x = estado_x;
        estado_pred_y = estado_y;
        
        float tiempo_pred = 0.2;
        int pasos_pred = static_cast<int>(tiempo_pred / dt);

        for (int i = 0; i < pasos_pred; i++) {
            estado_pred_x = A * estado_pred_x + B * U_x(1);
            estado_pred_y = A * estado_pred_y + B * U_y(1);
        }

        std::stringstream ss1;
        ss1 << "estado_x predicho:\n";
        ss1 << estado_pred_x(0)  << "\n";
        ss1 << estado_pred_x(1)  << "\n";

        //RCLCPP_INFO(this->get_logger(), "%s", ss1.str().c_str());


        P = (Eigen::Matrix2f::Identity() - K) * P;
    }
}

// SE IMPLEMENTA LA FUNCIÓN DE CALCULO DE LA VELOCIDAD SEGÚN EL ERROR
float mov_PTU::calcularPID(float error, float &error_prev, float &integral, float Kp, float Ki, float Kd, float dt)
{
    /*
     En esta función se implementa el funcionamiento de un controlador PID, de forma que según el valor de las ganancias y del error, se 
     calcula la velocidad que corresponde
    */
    float proporcional = Kp * error;
    if (error<3)
    {
        integral=0;
    }
    integral += Ki * error * dt;
    float derivativo = Kd * (error - error_prev)/dt;

    float salida = proporcional + integral + derivativo;

    //Limitar velocidad
    if (salida > v_max) salida = v_max;
    if (salida < -v_max) salida = -v_max;

    if (salida > -v_min and salida < 0 )
    {
        salida = 0.0;
    }
    else if(salida > 0 and salida < v_min)
    {
        salida = 0.0;
    }
    
    return salida;
}

// SE IMPLEMENTA LA FUNCIÓN PRINCIPAL DEL CÓDIGO QUE PERMITE MOVER LA PTU
void mov_PTU::mover_PTU()
{
    /*
     En esta función se implementa la acción principal del nodo que es el envío de comandos de movimiento a la PTU
     Para ello, lo primero que se hace es calcular el tiempo entre iteraciones del bucle, para conocer el tiempo de muestreo
     real.
     El siguiente paso es ajustar las ganancias del PID en función del comportamiento obtenido.
     Con estas ganancias se aplican los filtros de Kalman y se obtiene una predicción del error con el que comandar la PTU, ya que así 
     el sistema puede adelantarse al error y corregirlo antes de que ocurra. 
     Se le añade un offset en píxeles según la distancia para bajar el error medio y se calcula la velocidad con la que comandar la PTU, a 
     la que se le añade además un offset de velocidad que siempre debe de estar por debajo de 0.024 rad/s, que es la velocidad mínima a la que
     se mueve la PTU.
     Se añade además una restricción de movimiento cuando se han alcanzado alguno de los límites de posición establecidos.
    */
    interbotix_xs_msgs::msg::JointGroupCommand accion;
    static float vel_x = 0.0;
    static float vel_y = 0.0;

    rclcpp::Time tiempo_actual = now();
    dt = (tiempo_actual - tiempo_anterior).seconds();
    //RCLCPP_INFO(this->get_logger(),"La frecuencia de muestreo es: %f",1/(dt));
    tiempo_anterior = tiempo_actual;

    int zone1, zone2;
    bool north1, north2;
    double x1, y1, x2, y2;

    GeographicLib::UTMUPS::Forward(latitude_hunter, longitude_hunter, zone1, north1, x1, y1);
    GeographicLib::UTMUPS::Forward(latitude_ptu, longitude_ptu, zone2, north2, x2, y2);

    double dx = x2 - x1;
    double dy = y2 - y1;
    distancia = std::sqrt(dx * dx + dy * dy);

    std_msgs::msg::Float32 Dist;
    Dist.data = distancia;
    Publicador_distancia -> publish(Dist);

    RCLCPP_INFO(this->get_logger(),"DISTANCIA: %f", distancia);

    ajustar_PID();

    aplicarFiltroKalman();

    //RCLCPP_INFO(this->get_logger(), "estado_x(0): %f, estado_y(0): %f", estado_x(0), estado_y(0));
    if(estado_pred_x(0)!=0)
    {
        estado_pred_x(0)=estado_pred_x(0)+0;
    }

    if(estado_pred_y(0)!=0)
    {
        estado_pred_y(0)=estado_pred_y(0)+0;
    }

    vel_x = 0.0 + calcularPID(estado_pred_x(0), error_prev_x, integral_x, Kp_x, Ki_x, Kd_x, dt);
    vel_y = 0.0 + calcularPID(estado_pred_y(0), error_prev_y, integral_y, Kp_y, Ki_y, Kd_y, dt);

    // vel_x = 0.0 + calcularPID(error_x, error_prev_x, integral_x, Kp_x, Ki_x, Kd_x, dt);
    // vel_y = 0.0 + calcularPID(error_y, error_prev_y, integral_y, Kp_y, Ki_y, Kd_y, dt);

    // Publicar comandos de movimiento
    accion.name = "turret";
    if (limite_antihorario==true)
    {
        if((-vel_x)>0)
        {
            vel_x=0.0;
        }
    }
    else if(limite_horario==true)
    {
        if((-vel_x)<0)
        {
            vel_x=0.0;
        }
    }

    if (limite_abajo==true)
    {
        if (vel_y>0)
        {
            vel_y=0.0;
        }
    }
    else if (limite_arriba==true)
    {
        if (vel_y<0)
        {
            vel_y=0.0;
        }
    }
    //vel_x = 0.0;
    //vel_y = 0.0;
    accion.cmd = {-vel_x, vel_y};
    Publicador->publish(accion);

    // Guardar errores anteriores
    error_prev_x = estado_pred_x(0);
    error_prev_y = estado_pred_y(0);

    RCLCPP_INFO(this->get_logger(), "Error X: %f, Vel X: %f | Error Y: %f, Vel Y: %f", error_x, vel_x, error_y, vel_y);
}

// SE IMPLEMENTA LA FUNCIÓN MAIN DEL NODO
int main(int argc, char *argv[])
{
    // Se coloca 10 hz de velocidad de ejecución puesto que se ha visto que esta es la frecuencia de transmisión de los mensajes de la PTU 
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mov_PTU>();

    auto logger = node->get_logger();
    logger.set_level(rclcpp::Logger::Level::Warn);

    rclcpp::Rate loop_rate(10.000);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        node->mover_PTU();
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
