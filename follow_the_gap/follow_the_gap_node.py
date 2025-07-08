import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_node')
        # Suscripción al LiDAR y publicación de comandos de dirección
        # El LiDAR publica en /scan y se publica la dirección en /drive

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        # Se surcibe a odometria para hacer el contador y el cronometro
        self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.check_vuelta,
            10
        )

        # Parámetros
        self.max_steering_angle = 2.0  # rad

        self.en_zona_meta = False
        self.contador_vueltas = 0     
        self.inicio_completado = False # Para no contar la salida como vuelta

        self.tiempo_ultima_vuelta = None
        self.tiempos_vueltas = []

    def preprocess_lidar(self, ranges):
        """
        Limpieza de datos del LiDAR:
        - NaN y -inf se reemplazan por 0.0 (obstáculo cercano)
        - +inf se reemplaza por 10.0 (máximo artificial)
        - Se recortan valores fuera del rango [0.0, 5.0]
        """

        proc_ranges = np.array(ranges) # Array con los datos del lidar

        # 1. Reemplazar valores especiales:
        proc_ranges = np.nan_to_num(proc_ranges, nan=0.0, posinf=10.0, neginf=0.0)

        # 2. Recortar valores mayores a 5.0
        proc_ranges = np.clip(proc_ranges, 0.0, 5.0)

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """
        Encuentra el gap (hueco) más largo en el array de distancias.

        Parámetro:
        - ranges: array procesado del LiDAR

        Retorna:
        - start_i, end_i: índices del inicio y fin del gap más largo
        """
        # Crea una máscara que filtra los puntos con distancia menor al umbral
        threshold = 3.8  # Distancia mínima para considerar "espacio libre"
        masked = np.ma.masked_where(free_space_ranges < threshold, free_space_ranges)

        # Agrupa regiones consecutivas que no están enmascaradas
        gaps = np.ma.clump_unmasked(masked)

        # Si no se encuentra ningún gap válido, usar todo el campo de visión
        if len(gaps) == 0:
            return 0, len(free_space_ranges) - 1  # Todo el array

        # Buscar el gap más largo
        largest_gap = max(gaps, key=lambda gap: gap.stop - gap.start)

        return largest_gap.start, largest_gap.stop - 1
    
    def find_best_point(self, start_i, end_i, ranges):
        """
        Selecciona el punto central del gap más largo como dirección objetivo.
        Esto evita sesgos cuando hay múltiples puntos con la misma distancia.

        Parámetros:
        - start_i: índice de inicio del gap
        - end_i: índice final del gap
        - ranges: array de distancias del LiDAR

        Retorna:
        - best_index: índice del punto central del gap
        """
        best_index = (start_i + end_i) // 2

        return best_index
    
    def lidar_callback(self, data):
        """
        === ALGORITMO PRINCIPAL DE NAVEGACIÓN ===
        1. Obtener datos del lidar
        2. Preprocesa los datos
        3. Crea una "burbuja de seguridad" alrededor del obstáculo más cercano para evitarlo
        4. Encuentra el gap más largo
        5. Elige el mejor punto dentro del gap
        6. Convierte el índice a ángulo
        7. Publica un mensaje de Ackermann para girar y avanzar
        """

        # 1. Obtener datos del LiDAR
        full_ranges = np.array(data.ranges)
        total_points = len(full_ranges)
        fov_points = 270  # Aproximadamente 67.5° de FOV si total = 1080 (1080 * 270/1080 = 25% del escaneo total)
        center = total_points // 2
        start_idx = center - (fov_points // 2)
        end_idx = center + (fov_points // 2)
        ranges = full_ranges[start_idx:end_idx]


        # 2. Preprocesar los datos
        proc_ranges = self.preprocess_lidar(ranges)

        # 3. Eliminar burbuja
        closest_idx = np.argmin(proc_ranges)
        bubble_radius = 5  # número de puntos alrededor del más cercano que se eliminan 
        start = max(0, closest_idx - bubble_radius)
        end = min(len(proc_ranges), closest_idx + bubble_radius)
        proc_ranges[start:end] = 0.0

        # 4. Encontrar el gap más largo
        start_i, end_i = self.find_max_gap(proc_ranges)

        # 5. Escoger el mejor punto (centro) dentro del gap
        best_i = self.find_best_point(start_i, end_i, proc_ranges)

        # 6. Calcular ángulo de dirección (0 = centro del LiDAR recortado)
        mid_idx = len(proc_ranges) // 2

        # Ajuste del ángulo considerando el recorte
        relative_index = best_i - mid_idx
        steering_angle = relative_index * data.angle_increment

        # Limitar el ángulo de giro si excede el máximo
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

        # Calcular velocidad dependiendo del ángulo
        abs_angle = abs(steering_angle)
        velocidad = np.interp(abs_angle, [0.0, self.max_steering_angle], [11.3, 1.0])

        # 7. Publicar el comando
        self.publish_drive(steering_angle, velocidad)

    def publish_drive(self, angle, speed):
        """
        Publica un mensaje AckermannDriveStamped con ángulo y velocidad.
        """
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(angle)
        drive_msg.drive.speed = float(speed)
        self.drive_pub.publish(drive_msg)

    def check_vuelta(self, odom_msg):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        # Zona de meta (No se usa una recta para evitar mas de una meta virtual en la pista)
        en_meta = (-1.4 <= (x + y) <= 1.4) and (-0.4 <= (x - y) <= 0.4)

        if en_meta and not self.en_zona_meta:
            if self.inicio_completado:
                self.contador_vueltas += 1

                # ⏱️ Calcular tiempo de vuelta
                ahora = time.time()
                if self.tiempo_ultima_vuelta is not None:
                    duracion = ahora - self.tiempo_ultima_vuelta
                    minutos = duracion / 60.0
                    self.tiempos_vueltas.append(minutos)
                    self.get_logger().info(
                        f"🏁 Vuelta {self.contador_vueltas} completada. Tiempo: {minutos:.3f} min"
                    )
                self.tiempo_ultima_vuelta = ahora

            else:
                self.inicio_completado = True
                self.tiempo_ultima_vuelta = time.time()  # ⏱️ Iniciar cronómetro

            self.en_zona_meta = True

        elif not en_meta:
            self.en_zona_meta = False

def main(args=None):
    rclpy.init(args=args)
    print("FollowTheGap Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()