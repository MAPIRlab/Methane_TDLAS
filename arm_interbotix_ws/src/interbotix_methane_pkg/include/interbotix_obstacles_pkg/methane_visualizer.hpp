#pragma once

// Inclusión de las librerías necesarias de ROS2 y mensajes utilizados
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "olfaction_msgs/msg/tdlas.hpp"
#include "std_msgs/msg/bool.hpp"

// Declaración de la clase MethaneVisualizer que hereda de rclcpp::Node
class MethaneVisualizer : public rclcpp::Node {
    public:
        // Constructor de la clase
        MethaneVisualizer();
    
    private:
        // Método para publicar un marcador de visualización
        void publish_marker();

        // Callback que se ejecuta al recibir un nuevo mapa de ocupación
        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

        // Callback para mensajes booleanos relacionados con el PTU
        void callback_ptu(const std_msgs::msg::Bool::SharedPtr msg);

        // Callback para mensajes booleanos relacionados con el Hunter
        void callback_hunter(const std_msgs::msg::Bool::SharedPtr msg);

        // Publicador para los mensajes de marcador de visualización
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

        // Suscriptor para el mapa de ocupación
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

        // Callback para los mensajes de concentración de metano (TDLAS)
        void tdlas_callback(const olfaction_msgs::msg::TDLAS::SharedPtr msg);

        // Suscriptor para los mensajes de concentración de metano
        rclcpp::Subscription<olfaction_msgs::msg::TDLAS>::SharedPtr concentration_sub_;

        // Temporizador para ejecutar tareas periódicas
        rclcpp::TimerBase::SharedPtr timer_;

        // Suscriptores para mensajes booleanos de PTU y Hunter
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Subscriptor_ptu;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Subscriptor_hunter;
    
        // Buffer y listener para transformaciones (TF2)
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
    
        // Almacena el mapa de ocupación actual
        nav_msgs::msg::OccupancyGrid current_map_;

        // Indica si se ha recibido el mapa
        bool map_received_;

        // Identificador para los marcadores
        int marker_id_;

        // Almacena la concentración actual de metano
        float current_concentration_;

        // Flags para saber si se ha recibido concentración y si está escaneando
        bool concentration_received_, escaneando;
};