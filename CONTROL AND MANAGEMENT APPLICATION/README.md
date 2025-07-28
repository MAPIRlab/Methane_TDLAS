# MethaneScan — Aplicación de Control y Gestión del Proceso de Detección del Metano con Tecnología TDLAS

Paquete **ROS 2 (ament_python)** que implementa una **aplicación de escritorio (PyQt + Qt WebEngine)** para **dirigir y supervisar** el proceso de detección con TDLAS: integra la interfaz gráfica, la comunicación con ROS 2, un *bridge* ROS↔MQTT y utilidades de simulación y registro de datos.

---

## Contenido y estructura relevante

```
methane_scan_ui_ws/
├─ src/
│  ├─ methane_scan/                  # Paquete ROS 2 (ament_python) con la app Qt
│  │  ├─ package.xml
│  │  ├─ setup.py                    # entry points: methane_scan_node, mqtt_ros_bridge_node
│  │  ├─ launch/
│  │  │  ├─ launch.py                # Orquesta el arranque de la app y los bridges
│  │  │  ├─ parameters.yaml          # Tópicos del nodo principal (methane_scan_node)
│  │  │  └─ mqtt_parameters.yaml     # Tópicos del bridge Python (mqtt_ros_bridge_node)
│  │  ├─ methane_scan/
│  │  │  ├─ methane_scan_node.py     # Nodo principal: GUI + lógica ROS
│  │  │  ├─ mqtt_ros_bridge_node.py  # Bridge ROS↔MQTT (Python)
│  │  │  ├─ controllers/             # Controladores (PTU, robot, TDLAS, core)
│  │  │  ├─ views/                   # Interfaz (páginas, componentes, estilos)
│  │  │  │  ├─ web/                  # Front web (Google Maps) integrado en Qt
│  │  │  │  │  ├─ index.html
│  │  │  │  │  └─ map.js             # Dibujo de trayectorias, áreas, PTU/Hunter, etc.
│  │  │  ├─ resources/               # Iconos/SVG y recursos Qt
│  │  │  ├─ qresources.qrc / *_rc.py
│  │  │  └─ __init__.py
│  │  └─ test/                       # Tests (controladores, integración, estilo)
│  └─ mqtt_bridge/                   # Paquete ROS 2 (ament_cmake) en C++ (MQTT bridge)
│     ├─ CMakeLists.txt, package.xml
│     ├─ include/mqtt_bridge/CMQTTmosquitto2.hpp
│     ├─ src/mqtt_bridge_node.cpp    # Nodo ROS↔MQTT (C++)
│     └─ launch/
│        ├─ mqtt_basic_robot.py      # Launch del bridge C++
│        └─ mqtt_basic_robot.yaml    # Parámetros del broker y topics
├─ experiments/ …                    # Sesiones grabadas (rosbag2: *.db3 + metadata.yaml)
├─ install/, log/ (artefactos de build)
└─ .vscode/, configuración de tests (pytest.ini, setup.cfg, .coveragerc)
```

---

## Qué hace este módulo

- **Interfaz gráfica (PyQt5 + Qt WebEngine):**
  - Ventana principal **`MainWindow`** con páginas: *Home*, *PTU Config*, *Robot Config*, *Simulation*, y componentes como *MapView*, *TitleBar*, *Toast*.
  - **Mapa satelital** embebido (Google Maps, `views/web/index.html` + `map.js`) para:
    - Dibujar **trayectorias**, **regiones** (rectángulos/polígonos), y visualizar **rutas** (beams).
    - Mostrar y actualizar marcadores de **PTU** y **Hunter**, centrar mapa, registrar *path* de Hunter.
  - Estilos **Light/Dark** mediante `*.qss`.

- **Integración ROS 2 (Python):**
  - Nodo **`methane_scan_node`** que gestiona señales/estado y suscripciones/publicaciones ROS.
  - Controladores dedicados:
    - `PTUController`, `RobotController`, `TDLASController` (actualizan UI, validan estados y publican eventos).
  - Manejo de **eventos/tiempos** y *signals* Qt para desacoplar callbacks ROS y la UI.

- **Puente ROS↔MQTT:**
  - **Python:** `mqtt_ros_bridge_node.py` (usa `std_msgs/Bool`, `std_msgs/String`, `diagnostic_msgs/KeyValue`).
  - **C++:** paquete `mqtt_bridge` con `mqtt_bridge_node.cpp` y *launch* propio.
  - `launch/launch.py` del paquete **methane_scan** orquesta el arranque de la app y, tras su inicialización, lanza el/los bridges con sus parámetros.

- **Reproducción y registro:**
  - Página de **Reproducción** (`views/pages/simulation_page.py`) con utilidades para reproducción/guardado.
  - Carpeta **`experiments/`** con **rosbag2** (`*.db3` + `metadata.yaml`) para pruebas/ensayos.

---

## Tópicos y parámetros

**`launch/parameters.yaml`** (nodo `methane_scan_node`):
```yaml
methane_scan_node:
  ros_parameters:
    TOPICS:
      ptu_ready: /PTU_ready
      hunter_position: /hunter_position
      tdlas_ready: /TDLAS_ready
      tdlas_data: /falcon/reading
      initialize_hunter: /initialize_hunter_params
      start_hunter: /start_simulation
      end_simulation: /end_simulation
      save_simulation: /save_simulation
      play_simulation: /data_playback
      start_stop_hunter: /start_stop_hunter
      ptu_position: /fix
      mqtt_connection_status: /connection_status
      mqtt_bridge_status: /mqtt_status
```

**`launch/mqtt_parameters.yaml`** (nodo `mqtt_ros_bridge_node`):
```yaml
mqtt_ros_bridge_node:
  ros_parameters:
    TOPICS:
      hunter_position: /hunter_position
      ptu_position: /fix
      mqtt2ros: /mqtt2ros
      ros2mqtt: /ros2mqtt
      initialize_hunter: /initialize_hunter_params
      start_stop_hunter: /start_stop_hunter
```

> En el *bridge* C++ (`src/mqtt_bridge/launch/mqtt_basic_robot.yaml`) se define el **broker** y los **subtopics** MQTT a escuchar/publicar. Revísalo antes de desplegar (contiene host/puerto/credenciales y lista de subtemas).

**Tipos de mensajes (por imports en el código):**
- `std_msgs/Bool`, `std_msgs/String`, `diagnostic_msgs/KeyValue`
- `sensor_msgs/NavSatFix` (posición PTU)
- `olfaction_msgs/TDLAS` (datos del sensor TDLAS)

---

## Requisitos observados

- **ROS 2** (paquetes *ament_python* y *ament_cmake* para `mqtt_bridge`).
- **Dependencias Python** (según `setup.py` e *imports* del código):
  - `PyQtWebEngine` (en `install_requires` del paquete)
  - `python-dotenv` (carga de `.env`)
  - **Usadas en el código:** `PyQt5`, `requests`, `pexpect`, `opencv-python` (`cv2`), `numpy`
- **Dependencias ROS** usadas por el código:
  - `rclpy`, `std_msgs`, `sensor_msgs`, `diagnostic_msgs`, `olfaction_msgs`
- **C++ (`mqtt_bridge`)**: `rclcpp`, `diagnostic_msgs`

> Nota: El archivo `.env` (en `methane_scan/views/..`) se carga para obtener `GOOGLE_MAPS_API_KEY`. Si no está definido, el mapa puede quedar limitado.

---

## Configuración

1. **Clave de Google Maps (opcional pero recomendada):**
   - Crea un archivo `.env` accesible por la app (la ventana principal lo busca y registra si lo encuentra).
   - Define la variable:
     ```env
     GOOGLE_MAPS_API_KEY=tu_clave
     ```

2. **Parámetros de tópicos ROS y MQTT:**
   - Ajusta `launch/parameters.yaml` y `launch/mqtt_parameters.yaml` del paquete **methane_scan**.
   - Revisa `src/mqtt_bridge/launch/mqtt_basic_robot.yaml` (broker, subtemas).

---

## Compilación e instalación

En la raíz del *workspace* (`methane_scan_ui_ws/`):

```bash
# Opcional: resolver dependencias ROS declaradas
rosdep install --from-paths src -y --ignore-src

# Compilar
colcon build

# Cargar el entorno
source install/setup.bash
```

---

## Ejecución

### Opción A — *Launch* recomendado
Inicia la aplicación y, tras su arranque, los bridges ROS↔MQTT con sus parámetros:

```bash
ros2 launch methane_scan launch.py
```

### Opción B — Nodos por separado
```bash
# Interfaz + lógica ROS
ros2 run methane_scan methane_scan_node

# Bridge ROS↔MQTT (Python)
ros2 run methane_scan mqtt_ros_bridge_node

# Bridge ROS↔MQTT (C++) — desde el paquete mqtt_bridge
ros2 launch mqtt_bridge mqtt_basic_robot.py
```

---

## Tests

En `src/methane_scan/test/` hay pruebas de estilo y funcionales (controladores, integración de señales, etc.).
Para ejecutarlas (con el entorno del workspace cargado):

```bash
colcon test --packages-select methane_scan
colcon test-result --verbose
```

---

## Notas y mantenimiento

- Los *launch files* usan eventos para lanzar los bridges **después** de iniciar `methane_scan_node`.
- La carpeta `experiments/` contiene **rosbag2** (`*.db3` + `metadata.yaml`) con datos reales/simulados para pruebas.
- Si el mapa no aparece o no se guardan las trayectorias, revisa la presencia de `GOOGLE_MAPS_API_KEY` en el `.env`, la conectividad web de Qt WebEngine y comprueba que ambos servicios (Google Maps y Google Maps Static) estén habilitados.

---

## Autoría

- **Maintainer**: `Alejandro Román Sánchez` — <alerosan16@gmail.com>
