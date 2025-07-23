#include "interbotix_obstacles_pkg/methane_visualizer.hpp"
using std::placeholders::_1;

// Constructor de la clase MethaneVisualizer
MethaneVisualizer::MethaneVisualizer()
: Node("methane_visualizer"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    marker_id_(0),
    map_received_(false),
    concentration_received_(false),
    current_concentration_(0.0f),
    escaneando(false)
{
        // Publicador para los marcadores de visualización
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("methane_line", 10);

        // Suscriptor para el tópico que indica el fin del movimiento del PTU
        Subscriptor_ptu = this->create_subscription<std_msgs::msg::Bool>(
                "/Fin_PTU", 10, std::bind(&MethaneVisualizer::callback_ptu, this, _1));

        // Suscriptor para el tópico que indica el inicio del escaneo
        Subscriptor_hunter = this->create_subscription<std_msgs::msg::Bool>(
                "/Start_Scanner", 10, std::bind(&MethaneVisualizer::callback_hunter, this, _1));

        // Suscriptor para la concentración de metano medida por el sensor
        concentration_sub_ = this->create_subscription<olfaction_msgs::msg::TDLAS>(
                "/falcon/reading", 10, std::bind(&MethaneVisualizer::tdlas_callback, this, _1));

        // Suscriptor para el mapa de ocupación
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map", 1, std::bind(&MethaneVisualizer::map_callback, this, _1));

        // Temporizador para publicar el marcador periódicamente
        timer_ = this->create_wall_timer(std::chrono::milliseconds(450),
                std::bind(&MethaneVisualizer::publish_marker, this));
}

// Callback para recibir la concentración de metano
void MethaneVisualizer::tdlas_callback(const olfaction_msgs::msg::TDLAS::SharedPtr msg) {
        current_concentration_ = static_cast<float>(msg->average_ppmxm) / 255.0f;
        concentration_received_ = true;
}

// Callback para recibir el mapa de ocupación
void MethaneVisualizer::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_map_ = *msg;
        map_received_ = true;
}

// Callback para el fin del movimiento del PTU
void MethaneVisualizer::callback_ptu(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
                escaneando = false;
        }
}

// Callback para el inicio del escaneo
void MethaneVisualizer::callback_hunter(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
                escaneando = true;
        }
}

// Función que publica el marcador de la dirección del metano
void MethaneVisualizer::publish_marker() {
        // Espera a tener el mapa y la concentración
        if (!map_received_ || !concentration_received_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for map or concentration...");
                return;
        }

        // Solo publica si está escaneando
        if (!escaneando) {
                return;
        }

        float concentration = current_concentration_;
        geometry_msgs::msg::Point p1, p2;

        try {
                // Obtiene la transformación entre el mapa y el sensor
                auto tf_stamped = tf_buffer_.lookupTransform("map", "falcon_link", tf2::TimePointZero);
                tf2::Transform tf;
                tf2::fromMsg(tf_stamped.transform, tf);

                // Calcula la dirección del rayo en el marco del mundo
                tf2::Vector3 laser_dir_sensor_frame(1.0, 0.0, 0.0);  // dirección hacia adelante en el frame del sensor
                tf2::Vector3 laser_dir_world = tf.getBasis() * laser_dir_sensor_frame;
                tf2::Vector3 sensor_origin_map = tf.getOrigin();

                // Punto de inicio del rayo
                p1.x = sensor_origin_map.x();
                p1.y = sensor_origin_map.y();
                p1.z = sensor_origin_map.z();

                // Raycasting en 3D para encontrar el primer obstáculo
                float max_range = 20.0;
                float step = 0.05;
                p2 = p1;

                for (float d = 0.0; d <= max_range; d += step) {
                        float x = p1.x + laser_dir_world.x() * d;
                        float y = p1.y + laser_dir_world.y() * d;
                        float z = p1.z + laser_dir_world.z() * d;

                        int mx = static_cast<int>((x - current_map_.info.origin.position.x) / current_map_.info.resolution);
                        int my = static_cast<int>((y - current_map_.info.origin.position.y) / current_map_.info.resolution);

                        // Si el punto está fuera del mapa, termina el raycasting
                        if (mx < 0 || my < 0 || mx >= static_cast<int>(current_map_.info.width) || my >= static_cast<int>(current_map_.info.height))
                                break;

                        int index = my * current_map_.info.width + mx;
                        // Si encuentra un obstáculo, termina el raycasting
                        if (current_map_.data[index] > 50) {
                                p2.x = x;
                                p2.y = y;
                                p2.z = z;
                                break;
                        }
                }

                // Imprime información sobre la línea y la concentración
                RCLCPP_INFO(this->get_logger(),
                        "Line from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f), concentration: %.2f",
                        p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, concentration);

                // Define el color del marcador según la concentración
                std_msgs::msg::ColorRGBA color;
                color.a = 1.0;
                if (concentration <= 0.5f) {
                        color.r = 2.0f * concentration;
                        color.g = 1.0f;
                        color.b = 0.0f;
                } else {
                        color.r = 1.0f;
                        color.g = 2.0f * (1.0f - concentration);
                        color.b = 0.0f;
                }

                // Crea y publica el marcador de la línea
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = this->now();
                marker.ns = "methane_direction";
                marker.id = marker_id_++;
                marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.lifetime = rclcpp::Duration(0, 0);

                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.02;
                marker.color = color;

                marker.points.push_back(p1);
                marker.points.push_back(p2);

                marker_pub_->publish(marker);

        } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "TF exception: %s", ex.what());
        }
}

// Función principal
int main(int argc, char **argv) {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<MethaneVisualizer>());
        rclcpp::shutdown();
        return 0;
}
