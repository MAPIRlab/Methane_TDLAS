"""
Marker Detector Scripts Package

Este paquete contiene los scripts principales para la detección de marcadores LED
y control de lentes motorizadas en el sistema TDLAS.

Módulos principales:
- marker_detector: Detección de marcadores LED usando OpenCV
- open_camera: Control de cámara con autofoco y zoom automático
- test: Script de pruebas con detección de verde y autofoco
- camera_node: Nodo ROS2 para integración con el sistema

Módulos de control de lentes (LensConnect):
- LensConnect_Controller: Controlador principal para lentes motorizadas
- LensCtrl: Control de zoom, foco e iris
- LensInfo: Información de parámetros de las lentes
- LensSetup: Configuración de velocidades y corrección de backlash
- LensAccess: Acceso y movimiento de lentes
- UsbCtrl: Control de comunicación USB
- SLABHIDtoSMBUS: Wrapper para comunicación HID-SMBus
- SLABHIDDevice: Control de dispositivos HID
- DevAddr: Direcciones de registros del dispositivo
- ConfigVal: Valores de configuración
- DefVal: Valores por defecto del sistema

Uso típico:
    from scripts.open_camera import start
    from scripts.marker_detector import main as detector_main
    from scripts.LensConnect_Controller import UsbConnect, ScanUsbWithLensInfo
"""

# Importaciones principales para facilitar el acceso
from .marker_detector import main as run_marker_detector
from .open_camera import start as start_camera
from .test import main as run_test

# Control de lentes
from .LensConnect_Controller import UsbConnect, ScanUsbWithLensInfo, UsbDisconnect
from .LensCtrl import (
    ZoomInit, ZoomMove, FocusInit, FocusMove, IrisInit, IrisMove,
    ZoomParameterReadSet, FocusParameterReadSet, IrisParameterReadSet
)

# Módulos de configuración
from . import DefVal as DV
from . import ConfigVal as CV
from . import DevAddr as DA

__version__ = "1.0.0"
__author__ = "TDLAS Detection Team"

# Lista de módulos exportados
__all__ = [
    # Funciones principales
    'run_marker_detector',
    'start_camera', 
    'run_test',
    
    # Control de lentes
    'UsbConnect',
    'ScanUsbWithLensInfo', 
    'UsbDisconnect',
    'ZoomInit',
    'ZoomMove',
    'FocusInit', 
    'FocusMove',
    'IrisInit',
    'IrisMove',
    'ZoomParameterReadSet',
    'FocusParameterReadSet', 
    'IrisParameterReadSet',
    
    # Módulos de configuración
    'DV',
    'CV', 
    'DA',
]

def get_version():
    """Retorna la versión del paquete de scripts"""
    return __version__

def list_available_scripts():
    """Lista todos los scripts disponibles"""
    scripts = {
        'marker_detector': 'Detección de marcadores LED con tracking',
        'open_camera': 'Control de cámara con autofoco automático', 
        'test': 'Script de pruebas con detección de color verde',
        'camera_node': 'Nodo ROS2 para integración con el sistema'
    }
    return scripts

# Información del paquete
print(f"📦 Marker Detector Scripts v{__version__} cargado")