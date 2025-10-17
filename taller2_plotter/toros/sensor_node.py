# sensor_node.py (Versión Corregida)

import rclpy
from rclpy.node import Node
# --- CAMBIO: Importar Float32 en lugar de String ---
from std_msgs.msg import Float32
import random

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        # --- CAMBIO: Publicar un Float32 en el tópico 'temperature' ---
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)
        self.timer = self.create_timer(1.0, self.publish_data)

    def publish_data(self):
        # --- CAMBIO: Crear y llenar un mensaje Float32 ---
        msg = Float32()
        # Generamos un valor numérico y lo asignamos
        msg.data = float(random.randint(20, 30))
        
        self.publisher_.publish(msg)
        # El log muestra el número, pero podemos añadir el °C para claridad
        self.get_logger().info(f'Publicando: {msg.data:.1f}°C')

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
