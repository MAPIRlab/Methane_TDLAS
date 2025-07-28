# marker_detector
[![Python](https://img.shields.io/badge/python-43.2%25-blue)](#)

Paquete **ROS 2 (ament_python)** que expone un *nodo* para abrir una cámara (vía **Vimba Python SDK – `vmbpy`**), controlar una **óptica/lente por HID** (bibliotecas `.so` incluidas) y **detectar un marcador luminoso** mediante umbralado de intensidad y morfología.  
El nodo publica la **posición del marcador** en el *topic* `/Info_Posicion` como un `std_msgs/String` con **JSON**, para su posterior utilización con .

> **Idioma del código**: mayoritariamente en español.  
> **Nodo**: `autofocus_node` (ejecutable `marker_detector`).

---

## Contenido y estructura relevante

```
detection_ws/
└─ src/
   └─ marker_detector/
      ├─ marker_detector/
      │  ├─ camera_node.py              # Nodo ROS 2: crea publisher y lanza la cámara
      │  └─ scripts/
      │     ├─ open_camera.py           # Lógica principal: cámara, detección, autofocus/zoom, GUI
      │     ├─ marker_detector.py       # Script auxiliar de detección (no es el entrypoint)
      │     ├─ libslabhiddevice.so.1.0  # Librerías HID incluidas
      │     ├─ libslabhidtosmbus.so.1.0 #
      │     ├─ Lens*.py, SLAB*.py, UsbCtrl.py, etc.  # Control de lente/HID
      │     └─ template_200.png
      ├─ package.xml
      ├─ setup.py                       # Instala el wheel vmbpy y marca .so como ejecutables
      ├─ setup.cfg
      └─ vmbpy-1.1.0-py3-none-linux_x86_64.whl
```
---

## Requisitos observados

- **ROS 2** con soporte para paquetes `ament_python` y **colcon**.
- **Vimba X SDK** para los controladores de la cámara. Para descargarlo pincha [aquí](https://www.alliedvision.com/en/products/software/vimba-x-sdk/#c13326).
  Para instalarlo correctamente, una vez descargado y descomprimido, debe dirigirse a la carpeta cti y ejecutar el archivo `Install_GenTL_Path.sh`. Si quieres modificar los parámetros de la cámara
  sin ejecutar nodos ni scripts, el SDK ofrece **VimbaXViewer**, que se puede encontrar en /bin.
- **Dependencias ROS** (según `package.xml`):  
  `rclpy`, `sensor_msgs`, `std_msgs`, `cv_bridge`, `image_transport`.
- **Python**:
  - `numpy`
  - `opencv-python` (se importa como `cv2`)
  - `vmbpy` (**incluido** como wheel `vmbpy-1.1.0-py3-none-linux_x86_64.whl`).
- **Bibliotecas nativas** (incluidas en el paquete):
  - `libslabhiddevice.so.1.0`
  - `libslabhidtosmbus.so.1.0`  
  Estas se usan desde los módulos `SLAB*`/`UsbCtrl.py` para el control HID de la lente.

> **Nota**: El paquete trae un `requirements.txt`con las dependencias vistas en el código son `numpy`, `opencv-python`. Para instalar vmbpy desde pip (para uso local sin ROS2), diríjase
a la carpeta de Vimba SDK, en el directorio api/python y ejecuta
```bash
pip install ./vmbpy-1.1.1-py3-none-manylinux_2_27_x86_64.whl 

---

## Compilación e instalación (colcon)

1. Coloca el paquete dentro de un *workspace* de ROS 2, por ejemplo:

   ```bash
   detection_ws/
   └─ src/
      └─ marker_detector/
   ```

2. Desde la **raíz del workspace**:

   ```bash
   # Compilar
   colcon build

   # Cargar el entorno del workspace
   source install/setup.bash
   ```
---

## Ejecución

Con el entorno del workspace cargado:

```bash
ros2 run marker_detector marker_detector
```
> **Nota**: Para que funcione correctamente, debes añadir las siguientes reglas UDEV:
```bash
sudo nano /etc/udev/rules.d/99-lensconnect.rules
```

> Reemplaza el contenido (si tiene) por esto:
```bash
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="8d08", \
        MODE="0666", GROUP="plugdev", TAG+="uaccess"
```

> Ejecuta lo siguiente para añadirlo
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG plugdev $USER   # añade tu usuario al grupo
newgrp plugdev   
```

**Comportamiento en ejecución (según código):**

- Se imprime por consola el estado (p. ej., *“🔌 Iniciando cámara…”*, *“✅ Resolución: …”*).
- Se abren ventanas de **visualización** (`"Vista"`, `"Mask"`).
- Publica en **`/Info_Posicion`** un `std_msgs/String` con **JSON**:
  ```json
  {
    "height": <alto_imagen>,
    "width":  <ancho_imagen>,
    "pos_x":  <centro_x_del_objetivo>,
    "pos_y":  <centro_y_del_objetivo>
  }
  ```
  Puedes observarlo con:
  ```bash
  ros2 topic echo /Info_Posicion
  ```

---

## Parámetros y constantes (dentro del código)

> No hay **parámetros ROS** declarados; los ajustes son **constantes** definidas en `open_camera.py`. Algunos relevantes:

- **Cámara y exposición** (líneas iniciales de `open_camera.py`):
  - `WIDTH, HEIGHT = 3840, 2160`
  - `EXPOSURE_TIME = 10000`  (µs)
  - Formato de pixel: `BayerRG8` (vía `vmbpy`)
  - `ExposureAuto = Off`, `GainAuto = Off`, `Gain = 0.0`, `IntensityControllerTarget = 50.0`

- **Zoom / foco**:
  - `ZOOM_STEP = 1000`, `ZOOM_MIN = 4111`, `ZOOM_MAX = 17911`
  - `current_zoom = 4111`
  - Tabla `LOOKUP_TABLE` con valores `far/near` por zoom para búsquedas de foco.
  - Control de zoom por teclado (↑/↓); autofocus/ajuste se desencadenan según estado del ROI.

- **Detección (máscara y ROI)**:
  - Umbral en canal V (HSV): `INTENSITY_MIN = 250`
  - Área mínima base: `MIN_AREA = 10000` (se reescala dinámicamente con el zoom).
  - Márgenes del ROI: `ROI_MARGIN = 500` px
  - Pérdida de ROI: `ROI_LOST = 15` *frames*
  - Ajustes dinámicos de área: `AREA_TOLERANCE = 0.15`, `AREA_MIN/AREA_MAX` (calculados en tiempo de ejecución).

- **Ventanas**: `"Vista"` y `"Mask"` se muestran redimensionadas para visualización.

> Para modificar el comportamiento, edita estas constantes en `marker_detector/scripts/open_camera.py` y recompila si es necesario.

---

## Topic expuesto

- **`/Info_Posicion`** (`std_msgs/String`):
  - **Carga útil**: un **JSON** con campos:
    - `height`: alto de la imagen procesada.
    - `width`: ancho de la imagen procesada.
    - `pos_x`: coordenada X del centro del marcador detectado (en píxeles).
    - `pos_y`: coordenada Y del centro del marcador detectado (en píxeles).

---


## Mantenimiento y notas

- **`setup.py`** contiene una instalación personalizada que:
  - Instala `vmbpy` desde el wheel **incluido** (`v1.1.0`, *Linux x86_64*).
  - Ajusta permisos de los `.so` HID incluidos.
- **`package.xml`**:
  - `description` y `license` están en **TODO**; conviene completarlos.
- **Dependencias Python**:
  - Asegúrate de que **`numpy`** y **`opencv-python (cv2)`** estén disponibles en el entorno donde se ejecuta el nodo.
- **Ejecutables/ventanas**:
  - El código abre ventanas de OpenCV; se sale con **ESC**.

---

## Autoría 

- **Maintainer**: `Alejandro Román Sánchez` — <alerosan16@gmail.com>
