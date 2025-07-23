#include "percepcion/detector_aruco.hpp"

DetectorColor::DetectorColor():Node("detector_color"), cap(2)
{
    Publicador = this->create_publisher<std_msgs::msg::String>("/Info_Posicion", 10);

    if (!cap.isOpened()) {
        std::cerr << "Error: No se pudo abrir la cámara." << std::endl;
    }

    width = 1280;
    height = 720;

    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    // Configurar el rango de color en HSV (modifica estos valores según el color deseado)
    low_HSV = cv::Scalar(70, 100, 100);   // Verde azulado bajo (más cerca del cian)
    high_HSV = cv::Scalar(100, 255, 255); // Verde azulado alto    
}

DetectorColor::~DetectorColor()
{
    printf("Saliendo de la ejecución...\n");
}

/**
 * @brief Analiza un frame capturado de la cámara para detectar objetos de un color específico.
 *
 * Este método realiza los siguientes pasos:
 * 1. Captura un frame desde la cámara.
 * 2. Convierte el frame al espacio de color HSV.
 * 3. Aplica un umbral para segmentar el color deseado.
 * 4. Realiza operaciones morfológicas para eliminar ruido.
 * 5. Encuentra los contornos de los objetos detectados.
 * 6. Selecciona el contorno de mayor área que supere un umbral mínimo.
 * 7. Calcula el centroide del objeto detectado más grande.
 * 8. Publica la posición del objeto detectado mediante un mensaje JSON.
 * 9. Dibuja un círculo en el centro del objeto detectado y en el centro de la imagen.
 * 10. Muestra la imagen procesada en una ventana.
 *
 * Si no se detecta ningún objeto válido, solo se dibuja el centro de la imagen.
 *
 * @note Requiere que las variables miembro `cap`, `low_HSV`, `high_HSV`, `width`, `height` y `Publicador` estén correctamente inicializadas.
 */
void DetectorColor::AnalizaFrame()
{
    cv::Mat frame, hsv, mask;
    json dato;

    dato["width"] = width;
    dato["height"] = height;

    cap >> frame;
    if (frame.empty()) {
        std::cerr << "Error: No se pudo capturar el frame." << std::endl;
        return;
    }

    // Convertir a espacio de color HSV
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // Aplicar umbral para detectar el color deseado
    cv::inRange(hsv, low_HSV, high_HSV, mask);

    // Operaciones morfológicas para eliminar ruido
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 2);  // Erosión + Dilatación
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 2); // Dilatación + Erosión

    // Encontrar contornos de los objetos detectados
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Point2f centro_im(width / 2, height / 2);

    double max_area = 0;
    cv::Point2f center_max;

    // Analizar los contornos encontrados y quedarse con el más grande
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > 500 && area > max_area) { // Filtrar objetos pequeños y elegir el de mayor área
            max_area = area;
            cv::Moments M = cv::moments(contour);
            if (M.m00 != 0) {
                center_max = cv::Point2f(M.m10 / M.m00, M.m01 / M.m00);
            }
        }
    }

    // Si se encontró un objeto válido, publicarlo y dibujarlo
    if (max_area > 500) {
        RCLCPP_INFO(this->get_logger(), "Coordenada x: %f, coordenada y: %f", center_max.x, center_max.y);
        dato["pos_x"] = center_max.x;
        dato["pos_y"] = center_max.y;

        std::string dato_str = dato.dump();
        auto msg = std_msgs::msg::String();
        msg.data = dato_str;
        Publicador->publish(msg);

        // Dibujar un círculo en el centro del objeto detectado más grande
        cv::circle(frame, center_max, 10, cv::Scalar(0, 0, 255), -1);
    }

    // Dibujar el centro de la imagen
    cv::circle(frame, centro_im, 10, cv::Scalar(255, 0, 0), -1);

    // Mostrar la imagen procesada
    cv::imshow("Detección de Color", frame);
    cv::waitKey(1);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    DetectorColor d;

    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok()) {
        d.AnalizaFrame();
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
