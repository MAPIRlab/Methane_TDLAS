# Detección de Fuentes de Metano mediante Tecnología TDLAS en Espacios Exteriores

[![GitHub Repository](https://img.shields.io/github/stars/MAPIRlab/Methane_TDLAS?style=social)](https://github.com/MAPIRlab/Methane_TDLAS)
[![Python](https://img.shields.io/badge/python-43.2%25-blue)](#)
[![C++](https://img.shields.io/badge/c++-40.9%25-blue)](#)
[![MATLAB](https://img.shields.io/badge/matlab-8.6%25-yellow)](#)

---

Este repositorio provee las herramientas necesarias para la detección de fuentes de metano utilizando tecnología TDLAS (Tunable Diode Laser Absorption Spectroscopy). El sistema está orientado a aplicaciones en espacios exteriores, empleando robótica móvil, sensórica avanzada y algoritmos de percepción y control.

---

## Tabla de Contenidos

- [Descripción General](#descripción-general)
- [Estructura del Repositorio](#estructura-del-repositorio)
  - [HUNTER](#hunter)
  - [TRACKING](#tracking)
  - [DETECTION](#detection)
- [Guía de Uso](#guía-de-uso)
- [Requisitos](#requisitos)
- [Contacto y Soporte](#contacto-y-soporte)

---

## Descripción General

Este proyecto surge como solución para la localización y seguimiento de fuentes de emisión de metano en exteriores, mediante la integración de sensores TDLAS, robótica móvil (Hunter), brazo robótico, cámara PTU y sistemas de percepción avanzados. Los algoritmos desarrollados permiten la navegación autónoma, adquisición de datos, procesamiento y visualización de resultados.

https://github.com/user-attachments/assets/16bcccb4-5e00-4620-a35c-ed0c9270b562

---

## Estructura del Repositorio

El repositorio está organizado en dos módulos principales, **HUNTER** y **TRACKING**, cada uno con sus correspondientes espacios de trabajo y funcionalidades específicas:

### HUNTER 

Está organizado en 3 espacios de trabajo diferentes: Methane_ws, arm_interbotix_ws y hunter_ws. En el ordenador de HUNTER hay un README que explica en profuncidad la utilidad de cada uno de estos tres espacios de trabajo.

- **Methane_ws**  
  Paquetes para el movimiento autónomo del robot y el adaptador MQTT, que es el que adapta los mensajes que llegan por los topics de ROS2 a las colecciones de MQTT y viceversa.

- **arm_interbotix_ws**  
  Control del brazo robótico Interbotix y la PTU (Pan-Tilt Unit) utilizada para la orientación de los sensores.

- **hunter_ws**  
  Incluye las transformaciones de sistemas de referencia de cada elemento del Hunter, el paquete de descripción del robot y el paquete de lanzamiento principal.

**Comandos de lanzamiento:**
- Exterior (incluye PTU):  
  ```bash
  ros2 launch hunter_launch_files hunter_methane_launch.py
  ```
- Interior (sin PTU):  
  ```bash
  ros2 launch hunter_launch_files hunter_complete_launch.py
  ```

> **Nota:** Todos los dispositivos requieren especificar el puerto de comunicaciones. Se han configurado reglas UDEV para una gestión automática y robusta. Si algún hardware presenta problemas de conexión, revise las reglas UDEV correspondientes.

---

### TRACKING

Incluye los códigos y herramientas para el seguimiento y cálculo de la posición de las fuentes de metano. Se organiza en tres espacios de trabajo:

- **Methane**  
  Drivers para los equipos (láser TDLAS, puntero láser, DeluoGPS). El paquete `lanzamiento` contiene los archivos de lanzamiento principales.
  El primero de ellos permite lanzar la parte de tracking: 

  ```bash
  ros2 launch launch_files tracking_launch.py
  ```

  El segundo es el encargado de lanzar todos los sensores que se utilizan:

    ```bash
  ros2 launch launch_files sensors_launch.py
  ```

  > **Nota:** Para cada uno de los dispostivos utilizados se han hecho las respectivas reglas UDEV que permiten que todo funcione de manera adecuada, por lo que al igual que en la parte del Hunter, si algo falla, será necesario entrar en los archivos de configuración de lanzamiento de los diferentes dispositivos que forman la estación de tracking.

- **graficas**  
  Nodo para la representación gráfica del error en el tracking y visualización de los resultados. Permite analizar el desempeño detectando píxeles de error y generando métricas.

- **percepcion**  
  Implementa el algoritmo principal de tracking en el nodo `mov_PTU`. Incluye también:
  - `detector_aruco`: Para detección del color verde.
  - `medir_retraso`: Para estudiar el retraso existente entre el inicio de una orden de funcionamiento y la respuesta del sistema.
  - `simulacion_movimiento`: Simula el comportamiento de la PTU sin tener que estar ésta en movimiento

El nodo `mov_PTU` posee un archivo de parámetros para configurar:
- Ganancias PID
- Valores de ajuste para la distancia focal según el encoder de zoom (`a5, a4, a3, a2, a1, a0`)
- Parámetros de calibración para apuntar el láser a la posición objetivo (`m_x, n_x, m_y, n_y`)
  
  > **Nota:** Esos parámetros están obtenidos de manera empírica, por lo que para obtener mejores resultados, es necesario obtener de nuevo estos parámetros con la mayor precisión posible. El método utilizado hasta ahora consiste en el reconocimiento de la posición que ocupa el punto verde (proveniente del sensor TDLAS) dentro de la imagen, de manera que ese es el píxel de la imagen con respecoto a la cual se debe de calcular el error.

---

### DETECTION

Módulo de **percepción visual** encargado de detectar en imagen el punto/marker procedente del sensor TDLAS y publicar su **posición en píxeles** para el resto del sistema.

> 🔎 **Documentación completa:** este módulo incluye su propio README con todos los detalles de instalación, dependencias, parámetros y flujo interno.  
> Consúltalo en `DETECTION/README.md` (imprescindible para instalar el SDK de cámara y librerías asociadas).

**Resumen de lo que hay:**
- *Workspace* ROS 2 con el paquete `marker_detector`.
- Nodo ejecutable `marker_detector` (nodo `autofocus_node`), que:
  - Abre una cámara **Allied Vision** vía **Vimba X SDK (`vmbpy`)**.
  - Controla la **óptica/zoom** mediante HID.
  - Detecta el marcador luminoso y calcula su **centro** en la imagen.
  - Publica la posición en el *topic* **`/Info_Posicion`** como `std_msgs/String` con JSON:  
    `{"height": H, "width": W, "pos_x": X, "pos_y": Y}`.
  - Muestra ventanas de visualización y permite control por teclado (flechas para zoom, `ESC` para salir).

**Puesta en marcha mínima** (consulta el README del módulo para requisitos previos y SDK):
```bash
# Dentro del workspace de DETECTION
colcon build 
source install/setup.bash
ros2 run marker_detector marker_detector
```

**Notas importantes:**
- La instalación del **Vimba X SDK** y librerías de cámara es obligatoria.  
  Sigue las instrucciones del README del módulo. Descarga oficial:  
  [Vimba X SDK de Allied Vision](https://www.alliedvision.com/en/products/software/vimba-x-sdk/#c13326).
- No se repiten aquí parámetros/constantes ni pasos de instalación específicos; todo está detallado en `DETECTION/README.md`. 

---

## Guía de Uso

1. **Instalación de dependencias:**  
   Consulte los README específicos en cada espacio de trabajo para instrucciones detalladas de instalación y configuración.

2. **Configuración de hardware:**  
   Asegúrese de tener conectados todos los dispositivos (Hunter, brazo robótico, PTU, sensores TDLAS, GPS, etc.) y que las reglas UDEV estén correctamente aplicadas.

3. **Lanzamiento de sistemas:**  
   Siga los comandos indicados en las secciones HUNTER y TRACKING según el entorno de trabajo (interior/exterior).

4. **Visualización y análisis:**  
   Utilice el espacio de trabajo `graficas` para analizar resultados y errores de tracking.

---

## Requisitos

- **Hardware**
  - Robot Hunter
  - Brazo robótico Interbotix
  - PTU (Pan-Tilt Unit)
  - Sensor TDLAS
  - GPS Deluo

- **Software**
  - ROS2 (Foxy/Humble)
  - Python, C++, MATLAB
  - MQTT Broker

- **Configuraciones**
  - Reglas UDEV para dispositivos USB y serie
  - Parámetros PID y calibraciones en archivos de configuración

---

## Contacto y Soporte

- Para dudas, sugerencias o soporte técnico puede abrir un [Issue](https://github.com/MAPIRlab/Methane_TDLAS/issues) en este repositorio.
- Contacto directo: [MAPIRlab](mailto:mapir@uma.es)
- Documentación adicional y tutoriales disponibles en los README de cada espacio de trabajo.

---

> Proyecto desarrollado por [MAPIRlab](https://www.mapir.uma.es/) - Universidad de Málaga  
> Utilities for detecting methane sources using TDLAS technology.
