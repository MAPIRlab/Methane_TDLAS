import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np

class ErrorPlotter(Node):
    """
    ErrorPlotter is a ROS2 node for real-time visualization and statistical analysis of positional errors.
    This class subscribes to a topic publishing JSON-formatted position information, computes the error
    between the detected position and the image center, and plots the error over time for both X and Y axes.
    It also provides statistical summaries and error distribution percentages, with outlier filtering.
    Attributes:
        subscription: ROS2 subscription to the '/Info_Posicion' topic.
        error_x_data (list): List of X-axis error values.
        error_y_data (list): List of Y-axis error values.
        time_data (list): List of time indices corresponding to each error measurement.
        counter (int): Counter for time steps.
        fig (matplotlib.figure.Figure): Figure object for plotting.
        ax1, ax2 (matplotlib.axes.Axes): Axes for X and Y error plots.
        btn_reset (matplotlib.widgets.Button): Button widget to reset the plot.
    Methods:
        listener_callback(msg): Callback for incoming messages; parses JSON, computes errors, and updates data lists.
        filtrar_outliers(data): Filters outliers from data using the interquartile range (IQR) method.
        calcular_estadisticas(data): Calculates mean, max, and min absolute errors after outlier filtering.
        calcular_porcentajes(data): Computes the percentage of errors within specified ranges (<5, 5-10, 10-15, >15).
        init_plot(): Initializes the matplotlib plot and reset button.
        reset_plot(event): Clears all error and time data, resetting the plot.
        update_plot(): Updates the plots and statistical summaries with the latest data.
    """
    def __init__(self):
        super().__init__('error_plotter')
        self.subscription = self.create_subscription(
            String,
            '/Info_Posicion',
            self.listener_callback,
            10)
        
        self.error_x_data = []
        self.error_y_data = []
        self.time_data = []
        self.counter = 0
        
        self.init_plot()
    
    def listener_callback(self, msg):
        mensaje_json = msg.data
        mensaje = json.loads(mensaje_json)
        
        width = mensaje["width"]
        height = mensaje["height"]
        
        centro_imagen_x = width / 2
        centro_imagen_y = height / 2
        
        error_x = mensaje["pos_x"] - centro_imagen_x
        error_y = mensaje["pos_y"] - centro_imagen_y
        
        self.error_x_data.append(error_x)
        self.error_y_data.append(error_y)
        self.time_data.append(self.counter)
        self.counter += 1
    
    def filtrar_outliers(self, data):
        """Filtra valores atípicos usando el rango intercuartil (IQR)."""
        if len(data) < 4:
            return np.array(data)
        
        Q1 = np.percentile(data, 25)
        Q3 = np.percentile(data, 75)
        IQR = Q3 - Q1

        lower_bound = Q1 - 1.5 * IQR
        upper_bound = Q3 + 1.5 * IQR

        return np.array([x for x in data if lower_bound <= x <= upper_bound])

    def calcular_estadisticas(self, data):
        if data:
            datos_filtrados = self.filtrar_outliers(data)
            if len(datos_filtrados) > 0:
                error_medio = np.mean(np.abs(datos_filtrados))
                error_max = np.max(np.abs(datos_filtrados))
                error_min = np.min(np.abs(datos_filtrados))
                return error_medio, error_max, error_min
        return 0, 0, 0
    
    def calcular_porcentajes(self, data):
        """Calcula el porcentaje de errores en distintos rangos."""
        if not data:
            return 0, 0, 0, 0
        
        abs_data = np.abs(data)
        total = len(abs_data)
        bajo_5 = np.sum(abs_data < 5) / total * 100
        entre_5_10 = np.sum((abs_data >= 5) & (abs_data < 10)) / total * 100
        entre_10_15 = np.sum((abs_data >= 10) & (abs_data < 15)) / total * 100
        sobre_15 = np.sum(abs_data >= 15) / total * 100
        
        return bajo_5, entre_5_10, entre_10_15, sobre_15
    
    def init_plot(self):
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(6, 6))
        plt.subplots_adjust(bottom=0.2)
        
        self.ax_reset = plt.axes([0.4, 0.05, 0.2, 0.075])
        self.btn_reset = Button(self.ax_reset, 'Reset')
        self.btn_reset.on_clicked(self.reset_plot)
        
        plt.show(block=False)
    
    def reset_plot(self, event):
        self.error_x_data.clear()
        self.error_y_data.clear()
        self.time_data.clear()
        self.counter = 0
    
    def update_plot(self):
        self.ax1.clear()
        self.ax2.clear()
        
        self.ax1.plot(self.time_data, self.error_x_data, label='Error X', color='b')
        self.ax2.plot(self.time_data, self.error_y_data, label='Error Y', color='r')

        error_x_medio, error_x_max, error_x_min = self.calcular_estadisticas(self.error_x_data)
        error_y_medio, error_y_max, error_y_min = self.calcular_estadisticas(self.error_y_data)
        
        p_x_5, p_x_5_10, p_x_10_15, p_x_15 = self.calcular_porcentajes(self.error_x_data)
        p_y_5, p_y_5_10, p_y_10_15, p_y_15 = self.calcular_porcentajes(self.error_y_data)
        
        self.ax1.set_title(f'Error X - Medio: {error_x_medio:.2f}, Máx Abs: {error_x_max:.2f}, Mín Abs: {error_x_min:.2f}\n'
                            f'<5px: {p_x_5:.1f}%, 5-10px: {p_x_5_10:.1f}%, 10-15px: {p_x_10_15:.1f}%, >15px: {p_x_15:.1f}%')
        self.ax2.set_title(f'Error Y - Medio: {error_y_medio:.2f}, Máx Abs: {error_y_max:.2f}, Mín Abs: {error_y_min:.2f}\n'
                            f'<5px: {p_y_5:.1f}%, 5-10px: {p_y_5_10:.1f}%, 10-15px: {p_y_10_15:.1f}%, >15px: {p_y_15:.1f}%')
        
        self.ax1.set_xlabel('Tiempo')
        self.ax1.set_ylabel('Error X')
        self.ax1.legend()
        
        self.ax2.set_xlabel('Tiempo')
        self.ax2.set_ylabel('Error Y')
        self.ax2.legend()
        
        self.fig.canvas.draw()
        plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = ErrorPlotter()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.update_plot()
    except KeyboardInterrupt:
        print("Cerrando nodo...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
