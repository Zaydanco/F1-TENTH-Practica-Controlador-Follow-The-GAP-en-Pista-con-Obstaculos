import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

from nav_msgs.msg import Odometry
import math


class GapFollower(Node):
    def __init__(self):
        super().__init__('gap_follower')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )    
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )    

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        # Parámetros principales para ajustar BrandsHatch
        self.max_lidar_distance = 10.0
        self.safety_distance = 1.7
        self.bubble_radius = 45

        self.max_steering_angle = 0.28

        self.speed_straight = 4.8
        self.speed_medium_turn = 2.4
        self.speed_sharp_turn = 1.2

        self.previous_steering = 0.0
        self.steering_smoothing = 0.80

        self.get_logger().info('Controlador Follow the Gap robusto iniciado')
        
        # Contador de vueltas
        self.start_x = None
        self.start_y = None

        self.lap_count = 0
        self.lap_times = []

        self.lap_start_time = None
        self.inside_start_zone = False
        self.has_left_start_zone = False

        self.start_zone_radius = 1.0
        self.minimum_lap_time = 8.0

    def preprocess_lidar(self, ranges):
        ranges = np.array(ranges, dtype=np.float32)

        # Reemplazar lecturas malas
        ranges[np.isnan(ranges)] = 0.0
        ranges[np.isinf(ranges)] = self.max_lidar_distance

        # Limitar distancias máximas
        ranges = np.clip(ranges, 0.0, self.max_lidar_distance)

        # Suavizado de lecturas del LiDAR
        kernel_size = 7
        kernel = np.ones(kernel_size) / kernel_size
        ranges = np.convolve(ranges, kernel, mode='same')

        return ranges

    def get_front_ranges(self, ranges, scan_msg):
        total_points = len(ranges)

        # Solo usamos 180 grados frontales
        front_angle_degrees = 90
        angle_increment = scan_msg.angle_increment

        points_per_degree = int((np.pi / 180.0) / angle_increment)
        front_points = front_angle_degrees * points_per_degree

        center = total_points // 2
        start_index = max(0, center - front_points)
        end_index = min(total_points, center + front_points)

        return ranges[start_index:end_index], start_index

    def apply_safety_bubble(self, front_ranges):
        closest_index = np.argmin(front_ranges)

        bubble_start = max(0, closest_index - self.bubble_radius)
        bubble_end = min(len(front_ranges), closest_index + self.bubble_radius)

        free_space = np.copy(front_ranges)
        free_space[bubble_start:bubble_end] = 0.0

        # Eliminar zonas demasiado cercanas
        free_space[free_space < self.safety_distance] = 0.0

        return free_space

    def find_max_gap(self, free_space):
        max_start = 0
        max_end = 0
        current_start = None

        for i in range(len(free_space)):
            if free_space[i] > 0.0:
                if current_start is None:
                    current_start = i
            else:
                if current_start is not None:
                    current_end = i
                    if current_end - current_start > max_end - max_start:
                        max_start = current_start
                        max_end = current_end
                    current_start = None

        if current_start is not None:
            current_end = len(free_space)
            if current_end - current_start > max_end - max_start:
                max_start = current_start
                max_end = current_end

        return max_start, max_end

    def find_best_point(self, free_space, gap_start, gap_end):
        gap = free_space[gap_start:gap_end]

        if len(gap) == 0:
            return None

        # En vez de ir al punto más lejano, vamos más hacia el centro del gap.
        # Esto evita que el auto se cierre demasiado contra una pared.
        gap_center = (gap_start + gap_end) // 2

        # Buscamos alrededor del centro del gap, no en los extremos.
        window_size = 20
        search_start = max(gap_start, gap_center - window_size)
        search_end = min(gap_end, gap_center + window_size)

        if search_end <= search_start:
            return gap_center

        local_gap = free_space[search_start:search_end]

        # Dentro de esa zona central, elegimos el punto con más distancia libre.
        best_local_index = np.argmax(local_gap)
        best_point = search_start + best_local_index

        return best_point
        
    def calculate_steering_angle(self, best_point, start_index, scan_msg):
        lidar_index = start_index + best_point

        steering_angle = scan_msg.angle_min + lidar_index * scan_msg.angle_increment

        steering_angle = np.clip(
            steering_angle,
            -self.max_steering_angle,
            self.max_steering_angle
        )

        # Suavizado para que no zigzaguee
        steering_angle = (
            self.steering_smoothing * self.previous_steering
            + (1.0 - self.steering_smoothing) * steering_angle
        )

        self.previous_steering = steering_angle

        return steering_angle

    def calculate_speed(self, steering_angle, front_ranges):
        abs_steering = abs(steering_angle)
        front_distance = front_ranges[len(front_ranges) // 2]
        max_front_distance = np.max(front_ranges)

        if front_distance < 1.4:
            return 0.7

        # Mientras más recto esté el volante, más rápido.
        steering_factor = 1.0 - min(abs_steering / self.max_steering_angle, 1.0)

        # Mientras más espacio libre haya al frente, más rápido.
        distance_factor = min(max_front_distance / 8.0, 1.0)

        # Combinamos ambos factores.
        speed_factor = 0.65 * steering_factor + 0.35 * distance_factor

        min_speed = 1.0
        max_speed = 5.0

        speed = min_speed + speed_factor * (max_speed - min_speed)

        # Seguridad extra en curvas fuertes.
        if abs_steering > 0.24:
            speed = min(speed, 1.4)
        elif abs_steering > 0.14:
            speed = min(speed, 2.8)

        return speed

    def publish_drive(self, speed, steering_angle):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(speed)
        drive_msg.drive.steering_angle = float(steering_angle)

        self.drive_pub.publish(drive_msg)
        
    def odom_callback(self, odom_msg):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        current_time = self.get_clock().now().nanoseconds / 1e9

        # Guardar la posición inicial automáticamente
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.lap_start_time = current_time
            self.get_logger().info(
                f'Posición inicial guardada: x={self.start_x:.2f}, y={self.start_y:.2f}'
            )
            return

        distance_to_start = math.sqrt(
            (x - self.start_x) ** 2 + (y - self.start_y) ** 2
        )

        currently_inside = distance_to_start < self.start_zone_radius

        # Detectar que el auto ya salió de la zona inicial
        if not currently_inside:
            self.has_left_start_zone = True

        # Contar vuelta cuando vuelve a entrar a la zona inicial
        if (
            currently_inside
            and not self.inside_start_zone
            and self.has_left_start_zone
        ):
            lap_time = current_time - self.lap_start_time

            if lap_time > self.minimum_lap_time:
                self.lap_count += 1
                self.lap_times.append(lap_time)

                best_lap = min(self.lap_times)

                self.get_logger().info(
                    f'Vuelta {self.lap_count} completada | '
                    f'Tiempo: {lap_time:.2f} s | '
                    f'Mejor vuelta: {best_lap:.2f} s'
                )

                self.lap_start_time = current_time
                self.has_left_start_zone = False

        self.inside_start_zone = currently_inside 
        
    def print_final_results(self):
        print('\n===== RESUMEN FINAL =====')

        if len(self.lap_times) == 0:
            print('No se completaron vueltas.')
            return

        best_lap = min(self.lap_times)
        best_lap_number = self.lap_times.index(best_lap) + 1

        print(f'Vueltas completadas: {self.lap_count}')
        print(f'Vuelta más rápida: vuelta {best_lap_number} con {best_lap:.2f} s')
        print('=========================\n')

    def scan_callback(self, scan_msg):
        ranges = self.preprocess_lidar(scan_msg.ranges)

        front_ranges, start_index = self.get_front_ranges(ranges, scan_msg)

        free_space = self.apply_safety_bubble(front_ranges)

        gap_start, gap_end = self.find_max_gap(free_space)

        best_point = self.find_best_point(free_space, gap_start, gap_end)

        if best_point is None:
            self.publish_drive(0.0, 0.0)
            return

        steering_angle = self.calculate_steering_angle(
            best_point,
            start_index,
            scan_msg
        )

        speed = self.calculate_speed(steering_angle, front_ranges)

        self.publish_drive(speed, steering_angle)


def main(args=None):
    rclpy.init(args=args)

    node = GapFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_final_results()
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
