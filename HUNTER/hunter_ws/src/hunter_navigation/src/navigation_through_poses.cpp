// navigation_through_poses.hpp
#ifndef NAVIGATION_THROUGH_POSES__HPP_
#define NAVIGATION_THROUGH_POSES__HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <olfaction_msgs/msg/tdlas.hpp>
#include <hunter_navigation/action/navigate_to_pose.hpp>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>

using json = nlohmann::json;
using NavigateToPose = hunter_navigation::action::NavigateToPose;

/**
 * @brief Nodo de navegación que recorre una serie de poses agrupadas por habitaciones,
 *        lee valores de metano y controla un escáner PTU para decidir la siguiente
 *        pose a visitar.
 */
class navigation_through_poses : public rclcpp::Node
{
public:
  /**
   * @brief Constructor: declara parámetros, parsea JSON de poses, suscribe y publica.
   */
  navigation_through_poses();

  /**
   * @brief Destructor: informa final de ejecución.
   */
  ~navigation_through_poses();

private:
  /**
   * @brief Callback para lecturas TDLAS: cuenta valores altos/bajos de metano
   *        y decide si la habitación contiene metano.
   * @param msg  Mensaje con la concentración media de metano.
   */
  void callback_TDLAS(const olfaction_msgs::msg::TDLAS::SharedPtr msg);

  /**
   * @brief Callback al recibir confirmación de movimiento PTU:
   *        actualiza índices de habitación y punto según detección de metano.
   * @param msg  Mensaje booleano de fin de escaneo PTU.
   */
  void callback_movimiento_ptu(const std_msgs::msg::Bool::SharedPtr msg);

  /**
   * @brief Callback al completar acción de navegar a pose:
   *        en caso de éxito, inicia escaneo PTU; en caso contrario, reintenta.
   * @param result  Resultado envuelto de la acción NavigateToPose.
   */
  void callback_movimiento_hunter(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result);

  // Almacenamiento de poses: cada vector interno es una habitación con sus puntos.
  std::vector<std::vector<geometry_msgs::msg::Pose>> poses_;

  // Índices de posición en la matriz de poses
  int indice_habitacion;
  int indice_punto;

  // Flags de estado
  bool habitacion_con_metano;  // si se detectó metano en la habitación actual
  bool escaneo_ptu;            // si el escaneo PTU está en curso o listo

  // Publicador para iniciar PTU
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr Publicador_inicio_ptu;

  // Suscriptores para eventos de la PTU y lecturas TDLAS
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Subscriptor_ptu;
  rclcpp::Subscription<olfaction_msgs::msg::TDLAS>::SharedPtr Subscriptor_TDLAS;

  // Cliente de acción para navegar a pose
  rclcpp_action::Client<NavigateToPose>::SharedPtr cliente_poses;
};

#endif // NAVIGATION_THROUGH_POSES__HPP_


// navigation_through_poses.cpp
#include "hunter_navigation/navigation_through_poses.hpp"
using std::placeholders::_1;

navigation_through_poses::navigation_through_poses()
: Node("navigation_through_poses")
{
  // Declarar y leer parámetro JSON con poses
  this->declare_parameter<std::string>("poses_json", "{}");
  std::string poses_str = this->get_parameter("poses_json").as_string();

  // Parsear JSON y capturar errores
  json parsed;
  try {
    parsed = json::parse(poses_str);
  } catch (json::parse_error &e) {
    RCLCPP_ERROR(this->get_logger(), "Error al parsear JSON: %s", e.what());
    return;
  }

  poses_.clear();  // asegurarse de empezar sin datos previos

  // Convertir cada habitación y punto del JSON en Pose
  for (const auto& [nombre_habitacion, puntos] : parsed.items()) {
    std::vector<geometry_msgs::msg::Pose> fila;
    for (const auto& [nombre_punto, datos] : puntos.items()) {
      geometry_msgs::msg::Pose pose;
      // Asignar posición y orientación desde el JSON
      pose.position.x = datos["x_p"];
      pose.position.y = datos["y_p"];
      pose.position.z = datos["z_p"];
      pose.orientation.w = datos["w_o"];
      pose.orientation.x = datos["x_o"];
      pose.orientation.y = datos["y_o"];
      pose.orientation.z = datos["z_o"];

      fila.push_back(pose);
    }
    poses_.push_back(fila);
  }

  // Mostrar en el log todas las poses cargadas
  for (size_t i = 0; i < poses_.size(); ++i) {
    RCLCPP_INFO(this->get_logger(), "Habitación [%zu]:", i);
    for (size_t j = 0; j < poses_[i].size(); ++j) {
      const auto& pose = poses_[i][j];
      RCLCPP_INFO(this->get_logger(),
        "  Punto [%zu]: Pos=(%.3f, %.3f, %.3f) Ori=(%.3f, %.3f, %.3f, %.3f)",
        j,
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
      );
    }
  }

  // Crear publicador y suscriptores
  Publicador_inicio_ptu = this->create_publisher<std_msgs::msg::Bool>("/Start_Scanner", 10);
  Subscriptor_ptu = this->create_subscription<std_msgs::msg::Bool>(
    "/Fin_PTU", 10,
    std::bind(&navigation_through_poses::callback_movimiento_ptu, this, _1)
  );
  Subscriptor_TDLAS = this->create_subscription<olfaction_msgs::msg::TDLAS>(
    "/falcon/reading", 10,
    std::bind(&navigation_through_poses::callback_TDLAS, this, _1)
  );

  // Cliente de acción para navegar a cada pose
  cliente_poses = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  // Inicializar contadores y flags
  indice_habitacion     = -1;
  indice_punto          = 0;
  habitacion_con_metano = false;
  escaneo_ptu           = false;
}

navigation_through_poses::~navigation_through_poses()
{
  // Informe al terminar la ejecución
  RCLCPP_ERROR(this->get_logger(), "FIN DE LA EJECUCION");
}

void navigation_through_poses::callback_TDLAS(const olfaction_msgs::msg::TDLAS::SharedPtr msg)
{
  static int contador_positivos = 0;
  static int contador_negativos = 0;
  uint8_t concentracion = msg->average_ppmxm;

  // Contar lecturas altas o bajas de metano
  if (concentracion > 75) {
    contador_positivos++;
  } else {
    contador_negativos++;
  }

  // Reset si demasiados bajos consecutivos
  if (contador_negativos > 6) {
    contador_positivos = 0;
    contador_negativos = 0;
  }

  // Detectar habitación con metano
  if ((contador_positivos > 2 || concentracion > 100) && !habitacion_con_metano && escaneo_ptu) {
    RCLCPP_INFO(this->get_logger(), "HAY METANO EN LA HABITACION");
    habitacion_con_metano = true;
    contador_negativos = 0;
    contador_positivos = 0;
  }
}

void navigation_through_poses::callback_movimiento_ptu(const std_msgs::msg::Bool::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "RECIBIDO MENSAJE DE LA PTU");
  escaneo_ptu = false;

  if (msg->data) {
    // Actualizar índices según detección de metano
    if (habitacion_con_metano) {
      indice_punto++;
      if (indice_punto > 1) {
        indice_habitacion++;
        indice_punto = 0;
      }
      habitacion_con_metano = false;
    } else {
      indice_habitacion++;
    }
  }

  RCLCPP_INFO(
    this->get_logger(),
    "VAMOS A LA HABITACION: %d, PUNTO: %d",
    indice_habitacion, indice_punto
  );

  // Preparar y enviar objetivo de navegación
  auto goal_msg = NavigateToPose::Goal();
  auto pose = poses_[indice_habitacion][indice_punto];

  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = now();
  goal_msg.pose.pose = pose;

  auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  options.result_callback = std::bind(
    &navigation_through_poses::callback_movimiento_hunter,
    this,
    std::placeholders::_1
  );

  cliente_poses->async_send_goal(goal_msg, options);
}

void navigation_through_poses::callback_movimiento_hunter(
  const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result
)
{
  std_msgs::msg::Bool inicio_ptu;

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(), "¡Llegó a la meta!");
    inicio_ptu.data = true;
    Publicador_inicio_ptu->publish(inicio_ptu);
    escaneo_ptu = true;
  } else {
    RCLCPP_WARN(this->get_logger(), "Falló al llegar a la meta");
    // Reintentar callback de PTU con 'false'
    auto fallo = std::make_shared<std_msgs::msg::Bool>();
    fallo->data = false;
    this->callback_movimiento_ptu(fallo);
  }
}

int main(int argc, char *argv[])
{
  // Iniciar ROS2 y establecer frecuencia de 10 Hz para la PTU
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navigation_through_poses>();
  node->get_logger().set_level(rclcpp::Logger::Level::Info);

  rclcpp::Rate loop_rate(10.0);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}