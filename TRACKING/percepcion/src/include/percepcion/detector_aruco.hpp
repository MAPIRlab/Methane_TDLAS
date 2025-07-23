// Incluye la biblioteca principal de ROS2 para C++
#include <rclcpp/rclcpp.hpp>
// Incluye la biblioteca principal de OpenCV
#include <opencv2/opencv.hpp>
// Incluye la biblioteca estándar de entrada/salida
#include <iostream>
// Incluye el tipo de mensaje String de ROS2
#include "std_msgs/msg/string.hpp"
// Incluye la biblioteca JSON para C++
#include <nlohmann/json.hpp> 

// Alias para facilitar el uso de la biblioteca JSON
using json = nlohmann::json;

// Declaración de la clase DetectorColor que hereda de rclcpp::Node (nodo de ROS2)
class DetectorColor : public rclcpp::Node
{
    public:
    // Constructor de la clase
    DetectorColor();
    // Destructor de la clase
    ~DetectorColor();

    // Método público para analizar un frame de video
    void AnalizaFrame();

    private:
    // Publicador de mensajes tipo String en ROS2
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Publicador;

    // Objeto para capturar video con OpenCV
    cv::VideoCapture cap;
    // Ancho del frame de video
    int width;
    // Alto del frame de video
    int height;

    // Rango inferior de color en el espacio HSV (ajustar según el color a detectar)
    cv::Scalar low_HSV;
    // Rango superior de color en el espacio HSV (ajustar según el color a detectar)
    cv::Scalar high_HSV;
};