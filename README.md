# DETECCIÓN DE FUENTES DE METANO MEDIANTE TECNOLOGÍA TDLAS EN ESPACIOS EXTERIORES
https://github.com/user-attachments/assets/16bcccb4-5e00-4620-a35c-ed0c9270b562

- [Explicación del repositorio](#explicacion-del-repositorio)
- [HUNTER](#hunter)

## Explicacion del repositorio
Este repositorio está organizado en cuatro carpetas diferentes, HUNTER en el que se encuentran todos los códigos desarrollados para el ordenador del robot, TRACKING, en el que se encuentran los códigos desarrollados para realizar el seguimiento del patrón, APP, en el que se encuentran todos los códigos para lanzar la aplicación desarrollada para esta tarea y DETECTION en el que se encuentran los códigos necesarios para el funcionamiento de la cámara que permite detectar la ubicación del patrón.

## HUNTER
Está organizado en 3 espacios de trabajo diferentes: Methane_ws, arm_interbotix_ws y hunter_ws. En el ordenador de HUNTER hay un README que explica en profuncidad la utilidad de cada uno de estos tres espacios de trabajo.
- Methane_ws
En este espacio de trabajo se encuentran los paquetes desarrollados para el movimiento del robot y el adaptador de MQTT, que es el que adapta los mensajes que llegan por los topics de ROS2 a las colecciones de MQTT y viceversa.
- arm_interbotix_ws
En este espacio de trabajo, se encuentra todo lo relacionado con el brazo robótico utilizado y el control de la PTU para el TFG realizado.
- hunter_ws
Este es el espacio de trabajo en el que se encuentran las tranformaciones de los sistemas de referencia de los elementos que forman parte del hunter (hunter_description), se encuentra un paquete que permite la navegación de hunter en interiores incluyendo un fichero de parámetros funcional de configuración de NAV2 (hunter_navigation). Por último, el paquete más importante es el llamado hunter_launch_files, en el que se encuentra el archivo de lanzamiento. Para exteriores, el comando a realizar es:

```ros2 launch hunter_launch_files hunter_methane_launch.py```

Para interiores, se lanza todo menos la PTU y el comando es:

```ros2 launch hunter_launch_files hunter_complete_launch.py```

Para el funcionamiento de cada uno de los dispositivos (brazo, ptu, ouster, etc) es necesario especificar el puerto de comunicaciones utilizado, se han utilizado reglas udev para todos y cada uno de los dispositivos, por lo que tal vez sea necesario entrar en los ficheros de configuración de cada equipo necesario para ver que esas reglas están correctamente definidas y que coinciden con las especificadas en los ficheros de configuración.
