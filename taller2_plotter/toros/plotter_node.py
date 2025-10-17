import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib
# Usar un backend que no requiera GUI para funcionar en Docker
matplotlib.use('Agg') 
import matplotlib.pyplot as plt
import os
# Importar deque para la ventana deslizante
from collections import deque



class PlotterNode(Node):
    def __init__(self):
        super().__init__('plotter_node')
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.listener_callback,
            10)
        
        # --- 1. TAMAÑO DE LA VENTANA DESLIZANTE ---
        # Define cuántos puntos de datos recientes quieres mostrar en el gráfico.
        self.window_size = 30 
        
        # Usamos deque con un tamaño máximo para gestionar la ventana automáticamente.
        self.temperatures = deque(maxlen=self.window_size)
        self.timestamps = deque(maxlen=self.window_size)
        
        self.start_time = self.get_clock().now()
        
        # --- 2. RUTA DEL VOLUMEN ---
        # Esta es la ruta dentro del contenedor que mapeaste desde tu PC.
        self.plot_dir = '/graphs'
        
        # Asegurarse de que el directorio existe al iniciar.
        if not os.path.exists(self.plot_dir):
            os.makedirs(self.plot_dir)
        
        self.get_logger().info(f"Directorio de salida listo en: '{self.plot_dir}'")

        # Temporizador para generar el gráfico cada 5 segundos.
        self.timer = self.create_timer(5.0, self.plot_data)
        self.get_logger().info(f'Plotter node iniciado. Mostrando una ventana de {self.window_size} puntos.')

    def listener_callback(self, msg):
        """Callback que se ejecuta al recibir un mensaje de temperatura."""
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
        
        # 'append' en un deque con maxlen automáticamente elimina el dato más antiguo si está lleno.
        self.temperatures.append(msg.data)
        self.timestamps.append(elapsed_time)
        
        self.get_logger().info(f'Dato recibido: {msg.data:.2f}°C. Muestras en ventana: {len(self.temperatures)}')

    def plot_data(self):
        """Genera y guarda el gráfico con los datos actuales de la ventana."""
        if not self.temperatures:
            self.get_logger().warn('Aún no hay datos para graficar.')
            return

        plt.figure(figsize=(10, 5))
        plt.plot(self.timestamps, self.temperatures, marker='o', linestyle='-', color='b')
        plt.title(f'Últimas {len(self.temperatures)} Muestras de Temperatura')
        plt.xlabel('Tiempo (segundos desde inicio)')
        plt.ylabel('Temperatura (°C)')
        plt.grid(True)
        
        # Guarda la imagen en la ruta del volumen.
        plot_path = os.path.join(self.plot_dir, 'sensor_plot.png')
        plt.savefig(plot_path)
        plt.close() # Es crucial cerrar la figura para no consumir memoria.
        self.get_logger().info(f'¡Gráfico actualizado y guardado en: {plot_path}!')

def main(args=None):
    rclpy.init(args=args)
    plotter_node = PlotterNode()
    rclpy.spin(plotter_node)
    plotter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()