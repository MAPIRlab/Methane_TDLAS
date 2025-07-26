# DETECCIÓN DE FUENTES DE METANO MEDIANTE TECNOLOGÍA TDLAS EN ESPACIOS EXTERIORES
https://github.com/user-attachments/assets/16bcccb4-5e00-4620-a35c-ed0c9270b562

- [Explicación del repositorio](#explicacion-del-repositorio)
- [HUNTER](#hunter)
- [TRACKING](#tracking)

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

## TRACKING
Se organiza en tres espacios de trabajo de nuevo: Methane, graficas y percepcion. 
- Methane
En este primer espacio de trabajo, se encuentran los drivers para los equipos utilizados, láser TDLAS, puntero láser y DeluoGPS. Además, existe un paquete llamado lanzamiento en el que se encuentran los dos launch que permiten lanzar todo el sistema. El primer de ellos permite lanzar la parte de tracking y el comando de lanzamiento es:

```ros2 launch launch_files tracking_launch.py```

El segundo es el encargado de lanzar todos los sensores que se utilizan y el comando de lanzamiento es:

```ros2 launch launch_files sensors_launch.py```

Para cada uno de los dispostivos utilizados se han hecho las respectivas reglas udev que permiten que todo funcione de manera adecuada, por lo que al igual que en la parte del Hunter, si algo falla, será necesario entrar en los archivos de configuración de lanzamiento de los diferentes dispositivos que forman la estación de tracking.
- graficas
Este espacio de trabajo únicamente posee un paquete con un nodo y su utilidad es para representar el error cometido a la hora de llevar a cabbo el tracking, detectando los píxeles de error para obtener una medida de la fiabilidad del sistema desarrollado.
- percepcion
En este paquete es donde se encuentra el algoritmo de tracking desarrollado, concretamente en el nodo ```mov_PTU```, además hay otros 3 nodos más, ```detector_aruco``` que se usa para detectar un color verde, ```medir_retraso``` para estudiar el restraso existente entre el inicio de una orden de funcionamiento y la respuesta del sistema y ```simulacion_movimiento```es un nodo que simula el comportamiento de la PTU sin que esta se mueva realmente.

El nodo de movimiento de la PTU ```mov_PTU```posee un fichero de parámetros en el que se deben de introducir todas las configuraciones necesarias, en él se encuentran las ganancias de los PIDs sintonizados para cada valor de zoom, (nótese que para cada valor de zoom es necesario sintonizar un PID diferentes).

Se encuentran además una seria de valores (a5, a4, a3, a2, a1 y a0) que permite ajustar el valor de la distancia focal en función del valor del encoder de zoom utilizado.

Por último, aparecen los parámetros de dos rectas (m_x, n_x y m_y, n_y) que se corresponden con las rectas de calibración que permiten apuntar con el láser a la posición en la que se encuentra el patrón. Esos parámetros están obtenidos de manera empírica, por lo que para obtener mejores resultados, es necesario obtener de nuevo estos parámetros con la mayor precisión posible. El método utilizado hasta ahora consiste en el reconocimiento de la posición que ocupa el punto verde (proveniente del sensor TDLAS) dentro de la imagen, de manera que ese es el píxel de la imagen con respecoto a la cual se debe de calcular el error.
