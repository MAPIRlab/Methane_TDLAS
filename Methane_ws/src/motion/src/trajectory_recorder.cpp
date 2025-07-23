#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>
#include <utility>
#include <string>
#include <mutex>
#include <iomanip>
#include <algorithm>

using json = nlohmann::json;

// Definición de la clase principal que hereda de rclcpp::Node
class TrajectoryRecorder : public rclcpp::Node
{
public:
  // Constructor: inicializa el nodo y las suscripciones
  TrajectoryRecorder()
  : Node("trajectory_recorder")
  {
    // Suscripción a mensajes de GPS
    subscription_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/hunter/fix", 10, std::bind(&TrajectoryRecorder::gps_callback, this, std::placeholders::_1));

    // Suscripción a mensajes JSON con la trayectoria deseada y punto PTU
    subscription_json_ = this->create_subscription<std_msgs::msg::String>(
      "/initialize_hunter_params", 10, std::bind(&TrajectoryRecorder::json_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Trajectory recorder started.");
  }

  // Destructor: al destruir el nodo, escribe el archivo KML
  ~TrajectoryRecorder()
  {
    write_kml_file();
  }

private:
  // Callback para recibir datos del GPS y almacenar todos los puntos de la trayectoria real
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    actual_trajectory_.push_back({msg->latitude, msg->longitude});
  }

  // Callback para recibir el JSON con la trayectoria deseada y el punto PTU
  void json_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    try {
      json json_msg = json::parse(msg->data);

      std::lock_guard<std::mutex> lock(mutex_);
      desired_trajectory_.clear();
      // Extrae los puntos de la trayectoria deseada
      for (const auto& point : json_msg["points"]) {
        double lat = point["latitude"];
        double lon = point["longitude"];
        desired_trajectory_.push_back({lat, lon});
      }
      // Extrae el punto PTU
      ptu_point_ = { json_msg["point_ptu"]["latitude_ptu"], json_msg["point_ptu"]["longitude_ptu"] };
      // Extrae la velocidad máxima
      max_vel_ = json_msg["vel"];

      RCLCPP_INFO(this->get_logger(), "Received JSON with %zu desired points.", desired_trajectory_.size());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Error al parsear JSON: %s", e.what());
    }
  }

  // Función para escribir el archivo KML con líneas (LineString) para cada trayectoria y el punto PTU
  void write_kml_file()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::ofstream kml_file("trajectory_lines.kml");
    if (!kml_file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "No se pudo abrir trajectory_lines.kml para escritura.");
      return;
    }
    
    // Configura la precisión de los decimales
    kml_file << std::fixed << std::setprecision(10);

    // Escribe el encabezado del archivo KML
    kml_file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    kml_file << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
    kml_file << "  <Document>\n";
    kml_file << "    <name>Trajectories as Lines</name>\n";

    // 1) Trayectoria deseada (línea roja)
    if (!desired_trajectory_.empty()) {
      kml_file << "    <Placemark>\n";
      kml_file << "      <name>Desired Trajectory</name>\n";
      kml_file << "      <Style>\n";
      kml_file << "        <LineStyle>\n";
      // Rojo en formato AABBGGRR: "ff0000ff"
      kml_file << "          <color>ff0000ff</color>\n";
      kml_file << "          <width>4</width>\n";
      kml_file << "        </LineStyle>\n";
      kml_file << "      </Style>\n";
      kml_file << "      <LineString>\n";
      kml_file << "        <tessellate>1</tessellate>\n";
      kml_file << "        <coordinates>\n";
      for (const auto & point : desired_trajectory_) {
        // KML requiere: longitud,latitud,altitud
        kml_file << "          " << point.second << "," << point.first << ",0\n";
      }
      kml_file << "        </coordinates>\n";
      kml_file << "      </LineString>\n";
      kml_file << "    </Placemark>\n";
    }
    
    // 2) Trayectoria real (línea verde)
    if (!actual_trajectory_.empty()) {
      kml_file << "    <Placemark>\n";
      kml_file << "      <name>Actual Trajectory</name>\n";
      kml_file << "      <Style>\n";
      kml_file << "        <LineStyle>\n";
      // Verde: "ff00ff00"
      kml_file << "          <color>ff00ff00</color>\n";
      kml_file << "          <width>4</width>\n";
      kml_file << "        </LineStyle>\n";
      kml_file << "      </Style>\n";
      kml_file << "      <LineString>\n";
      kml_file << "        <tessellate>1</tessellate>\n";
      kml_file << "        <coordinates>\n";
      for (const auto & point : actual_trajectory_) {
        kml_file << "          " << point.second << "," << point.first << ",0\n";
      }
      kml_file << "        </coordinates>\n";
      kml_file << "      </LineString>\n";
      kml_file << "    </Placemark>\n";
    }
    
    // 3) Punto PTU (marcador azul)
    kml_file << "    <Placemark>\n";
    kml_file << "      <name>PTU Point</name>\n";
    kml_file << "      <Style>\n";
    kml_file << "        <IconStyle>\n";
    // Azul: "ffff0000"
    kml_file << "          <color>ffff0000</color>\n";
    kml_file << "          <scale>1.3</scale>\n";
    kml_file << "          <Icon>\n";
    kml_file << "            <href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>\n";
    kml_file << "          </Icon>\n";
    kml_file << "        </IconStyle>\n";
    kml_file << "      </Style>\n";
    kml_file << "      <Point>\n";
    kml_file << "        <coordinates>" << ptu_point_.second << "," << ptu_point_.first << ",0</coordinates>\n";
    kml_file << "      </Point>\n";
    kml_file << "    </Placemark>\n";

    // Cierre del documento KML
    kml_file << "  </Document>\n";
    kml_file << "</kml>\n";
    kml_file.close();

    RCLCPP_INFO(this->get_logger(), "Archivo KML escrito: trajectory_lines.kml (con %zu puntos reales).", actual_trajectory_.size());
  }

  // Suscripciones
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_gps_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_json_;

  // Almacenamiento de trayectorias y punto PTU
  std::vector<std::pair<double, double>> actual_trajectory_;   // Trayectoria real (GPS)
  std::vector<std::pair<double, double>> desired_trajectory_;  // Trayectoria deseada (JSON)
  std::pair<double, double> ptu_point_;                        // Punto PTU
  double max_vel_{0.0};                                        // Velocidad máxima

  std::mutex mutex_; // Mutex para proteger el acceso a los datos compartidos
};

// Función principal: inicializa ROS2, crea el nodo y lo ejecuta
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryRecorder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
