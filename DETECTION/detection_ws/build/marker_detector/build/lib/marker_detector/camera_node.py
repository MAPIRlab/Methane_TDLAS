import rclpy
from rclpy.node import Node
from marker_detector.scripts.open_camera import start
from std_msgs.msg import String

class CameraNode(Node):
    def __init__(self):
        super().__init__('autofocus_node')
        self.get_logger().info("AutoFocusNode iniciado")
        
        self.publisher = self.create_publisher(
            String,
            '/Info_Posicion',
            10
        )
        

    def run(self):
        # ejecuta la lógica del main() aquí, posiblemente en hilos
        pass

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    node.run()
    start(node)  # Llama a la función para abrir la cámara
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()