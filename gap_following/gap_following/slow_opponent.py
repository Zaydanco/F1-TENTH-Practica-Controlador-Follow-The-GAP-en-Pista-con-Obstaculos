import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class SlowOpponent(Node):
    def __init__(self):
        super().__init__('slow_opponent')

        # El oponente utiliza su propio LiDAR.
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/opp_scan',
            self.scan_callback,
            10
        )

        # Los comandos del oponente se publican en /opp_drive.
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/opp_drive',
            10
        )

        # Parámetros de seguridad.
        self.max_lidar_distance = 10.0
        self.safety_distance = 1.6
        self.bubble_radius = 45

        self.max_steering_angle = 0.28

        # El auto principal llega aproximadamente hasta 5 m/s.
        # El oponente quedará limitado a 2 m/s.
        self.min_speed = 0.7
        self.max_speed = 2.0

        # Límites adicionales para curvas.
        self.medium_turn_speed = 1.3
        self.sharp_turn_speed = 0.9

        # Suavizado del volante.
        self.previous_steering = 0.0
        self.steering_smoothing = 0.80

        self.get_logger().debug(
            'Controlador del vehículo oponente lento iniciado'
        )

    def preprocess_lidar(self, ranges):
        ranges = np.array(ranges, dtype=np.float32)

        # Limpiar lecturas inválidas.
        ranges[np.isnan(ranges)] = 0.0
        ranges[np.isinf(ranges)] = self.max_lidar_distance

        ranges = np.clip(
            ranges,
            0.0,
            self.max_lidar_distance
        )

        # Suavizar el LiDAR.
        kernel_size = 7
        kernel = np.ones(kernel_size) / kernel_size
        ranges = np.convolve(
            ranges,
            kernel,
            mode='same'
        )

        return ranges

    def get_front_ranges(self, ranges, scan_msg):
        total_points = len(ranges)

        # Analizar 90 grados hacia cada lado:
        # 180 grados frontales en total.
        front_angle_degrees = 90

        points_per_degree = int(
            (np.pi / 180.0) / scan_msg.angle_increment
        )

        front_points = front_angle_degrees * points_per_degree

        center = total_points // 2

        start_index = max(
            0,
            center - front_points
        )

        end_index = min(
            total_points,
            center + front_points
        )

        return ranges[start_index:end_index], start_index

    def apply_safety_bubble(self, front_ranges):
        closest_index = np.argmin(front_ranges)

        bubble_start = max(
            0,
            closest_index - self.bubble_radius
        )

        bubble_end = min(
            len(front_ranges),
            closest_index + self.bubble_radius
        )

        free_space = np.copy(front_ranges)

        # Anular el espacio alrededor del obstáculo más cercano.
        free_space[bubble_start:bubble_end] = 0.0

        # Anular zonas demasiado cercanas.
        free_space[
            free_space < self.safety_distance
        ] = 0.0

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

                    if (
                        current_end - current_start
                        > max_end - max_start
                    ):
                        max_start = current_start
                        max_end = current_end

                    current_start = None

        # Comprobar si el último gap llega hasta el final.
        if current_start is not None:
            current_end = len(free_space)

            if (
                current_end - current_start
                > max_end - max_start
            ):
                max_start = current_start
                max_end = current_end

        return max_start, max_end

    def find_best_point(
        self,
        free_space,
        gap_start,
        gap_end
    ):
        gap = free_space[gap_start:gap_end]

        if len(gap) == 0:
            return None

        # Preferir el centro del espacio libre.
        gap_center = (gap_start + gap_end) // 2

        window_size = 20

        search_start = max(
            gap_start,
            gap_center - window_size
        )

        search_end = min(
            gap_end,
            gap_center + window_size
        )

        if search_end <= search_start:
            return gap_center

        local_gap = free_space[search_start:search_end]

        best_local_index = np.argmax(local_gap)

        return search_start + best_local_index

    def calculate_steering(
        self,
        best_point,
        start_index,
        scan_msg
    ):
        lidar_index = start_index + best_point

        steering_angle = (
            scan_msg.angle_min
            + lidar_index * scan_msg.angle_increment
        )

        steering_angle = np.clip(
            steering_angle,
            -self.max_steering_angle,
            self.max_steering_angle
        )

        # Suavizar el cambio de dirección.
        steering_angle = (
            self.steering_smoothing
            * self.previous_steering
            + (1.0 - self.steering_smoothing)
            * steering_angle
        )

        self.previous_steering = steering_angle

        return steering_angle

    def calculate_speed(
        self,
        steering_angle,
        front_ranges
    ):
        abs_steering = abs(steering_angle)

        center_index = len(front_ranges) // 2
        front_distance = front_ranges[center_index]

        # Muy cerca de una pared u obstáculo.
        if front_distance < 1.4:
            return 0.6

        # Curva fuerte.
        if abs_steering > 0.22:
            return self.sharp_turn_speed

        # Curva media.
        if abs_steering > 0.12:
            return self.medium_turn_speed

        # Recta o curva muy suave.
        return self.max_speed

    def publish_drive(
        self,
        speed,
        steering_angle
    ):
        drive_msg = AckermannDriveStamped()

        drive_msg.drive.speed = float(speed)
        drive_msg.drive.steering_angle = float(
            steering_angle
        )

        self.drive_pub.publish(drive_msg)

    def scan_callback(self, scan_msg):
        ranges = self.preprocess_lidar(
            scan_msg.ranges
        )

        front_ranges, start_index = (
            self.get_front_ranges(
                ranges,
                scan_msg
            )
        )

        free_space = self.apply_safety_bubble(
            front_ranges
        )

        gap_start, gap_end = self.find_max_gap(
            free_space
        )

        best_point = self.find_best_point(
            free_space,
            gap_start,
            gap_end
        )

        if best_point is None:
            self.publish_drive(0.0, 0.0)
            return

        steering_angle = self.calculate_steering(
            best_point,
            start_index,
            scan_msg
        )

        speed = self.calculate_speed(
            steering_angle,
            front_ranges
        )

        self.publish_drive(
            speed,
            steering_angle
        )


def main(args=None):
    rclpy.init(args=args)

    node = SlowOpponent()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
