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
        
        self.opp_odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/opp_odom',
            self.opp_odom_callback,
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

        self.get_logger().debug('Controlador Follow the Gap robusto iniciado')
        
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
        
        # Datos del vehículo principal
        self.ego_x = None
        self.ego_y = None
        self.ego_yaw = None
        self.ego_speed = 0.0

        # Datos del vehículo oponente
        self.opp_x = None
        self.opp_y = None
        self.opp_speed = 0.0

        # Posición del oponente respecto al auto principal
        self.relative_opp_x = None
        self.relative_opp_y = None

        # Evita llenar la terminal de mensajes
        self.last_opponent_log_time = 0.0
        
        # Estado del adelantamiento
        self.overtaking = False
        
        # Fase previa al adelantamiento:
        # el auto comienza a desplazarse antes de alcanzar al oponente.
        self.preparing_overtake = False

        # 1 significa izquierda y -1 significa derecha
        self.overtake_direction = 0

        self.overtake_start_time = 0.0
        self.last_overtake_end_time = -10.0

        # Empezar a preparar el adelantamiento con anticipación
        self.pre_overtake_start_distance = 9.0

        # Distancia para pasar de preparación a adelantamiento completo
        self.overtake_trigger_distance = 6.8

        # Finalizar cuando el oponente quede detrás
        self.overtake_finish_distance = -1.5

        # El oponente debe estar aproximadamente en nuestra trayectoria
        self.overtake_lateral_limit = 1.6

        self.overtake_timeout = 9.0
        self.overtake_cooldown = 2.0

        # Espacios mínimos permitidos
        self.overtake_min_side_clearance = 0.95
        self.overtake_abort_side_clearance = 0.55

        # Velocidad máxima durante la preparación
        self.pre_overtake_speed_limit = 3.8

        # Velocidades durante el adelantamiento completo
        self.overtake_min_speed = 2.8
        self.overtake_speed_limit = 3.6

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
    
    def get_side_clearance(self, front_ranges, scan_msg):
        center = len(front_ranges) // 2
        angle_increment = abs(scan_msg.angle_increment)

        # Evaluar una zona más próxima a la trayectoria del vehículo.
        # Antes se comprobaban sectores entre 15° y 55°.
        inner_offset = max(
            1,
            int(math.radians(5.0) / angle_increment)
        )

        outer_offset = max(
            inner_offset + 1,
            int(math.radians(30.0) / angle_increment)
        )

        right_start = max(
            0,
            center - outer_offset
        )

        right_end = max(
            0,
            center - inner_offset
        )

        left_start = min(
            len(front_ranges),
            center + inner_offset
        )

        left_end = min(
            len(front_ranges),
            center + outer_offset
        )

        right_sector = front_ranges[
            right_start:right_end
        ]

        left_sector = front_ranges[
            left_start:left_end
        ]

        def calculate_clearance(sector):
            if len(sector) == 0:
                return 0.0

            valid_values = sector[
                sector > 0.05
            ]

            if len(valid_values) == 0:
                return 0.0

            # Un percentil un poco más permisivo evita que unas pocas
            # lecturas cercanas bloqueen todo el adelantamiento.
            return float(
                np.percentile(valid_values, 55)
            )

        left_clearance = calculate_clearance(
            left_sector
        )

        right_clearance = calculate_clearance(
            right_sector
        )

        return left_clearance, right_clearance
        
    def update_overtake_state(
        self,
        front_ranges,
        scan_msg
    ):
        if (
            self.relative_opp_x is None
            or self.relative_opp_y is None
        ):
            return

        current_time = (
            self.get_clock().now().nanoseconds / 1e9
        )

        left_clearance, right_clearance = (
            self.get_side_clearance(
                front_ranges,
                scan_msg
            )
        )

        # -------------------------------------------------
        # 1. Adelantamiento completo en ejecución
        # -------------------------------------------------
        if self.overtaking:
            opponent_is_behind = (
                self.relative_opp_x
                < self.overtake_finish_distance
            )

            maneuver_timed_out = (
                current_time - self.overtake_start_time
                > self.overtake_timeout
            )

            if self.overtake_direction > 0:
                selected_clearance = left_clearance
            else:
                selected_clearance = right_clearance

            selected_side_blocked = (
                selected_clearance
                < self.overtake_abort_side_clearance
            )

            if (
                opponent_is_behind
                or maneuver_timed_out
                or selected_side_blocked
            ):
                if opponent_is_behind:
                    reason = 'oponente superado'
                elif selected_side_blocked:
                    reason = 'lado seleccionado bloqueado'
                else:
                    reason = 'tiempo máximo alcanzado'

                self.get_logger().debug(
                    f'Adelantamiento finalizado: {reason}'
                )

                self.overtaking = False
                self.preparing_overtake = False
                self.overtake_direction = 0
                self.last_overtake_end_time = current_time

            return

        # -------------------------------------------------
        # 2. Esperar después de terminar una maniobra
        # -------------------------------------------------
        if (
            current_time - self.last_overtake_end_time
            < self.overtake_cooldown
        ):
            return

        opponent_is_ahead = (
            1.2
            < self.relative_opp_x
            < self.pre_overtake_start_distance
        )

        opponent_is_in_lane = (
            abs(self.relative_opp_y)
            < self.overtake_lateral_limit
        )

        # Si el oponente ya no está delante, cancelar preparación.
        if (
            not opponent_is_ahead
            or not opponent_is_in_lane
        ):
            self.preparing_overtake = False
            self.overtake_direction = 0
            return

        # -------------------------------------------------
        # 3. Preparación ya activa
        # -------------------------------------------------
        if self.preparing_overtake:
            if self.overtake_direction > 0:
                selected_clearance = left_clearance
                opposite_clearance = right_clearance
            else:
                selected_clearance = right_clearance
                opposite_clearance = left_clearance

            # Si el lado elegido se cierra, probar el contrario.
            if (
                selected_clearance
                < self.overtake_abort_side_clearance
            ):
                if (
                    opposite_clearance
                    >= self.overtake_min_side_clearance
                ):
                    self.overtake_direction *= -1
                else:
                    self.preparing_overtake = False
                    self.overtake_direction = 0
                    return

            # Al acercarse, comenzar el adelantamiento completo.
            if (
                self.relative_opp_x
                < self.overtake_trigger_distance
            ):
                self.preparing_overtake = False
                self.overtaking = True
                self.overtake_start_time = current_time

                self.get_logger().debug(
                    f'Adelantamiento completo iniciado | '
                    f'oponente a {self.relative_opp_x:.2f} m'
                )

            return

        # -------------------------------------------------
        # 4. Iniciar la preparación anticipada
        # -------------------------------------------------
        best_clearance = max(
            left_clearance,
            right_clearance
        )

        if (
            best_clearance
            < self.overtake_min_side_clearance
        ):
            return

        if left_clearance >= right_clearance:
            self.overtake_direction = 1
            selected_side = 'izquierda'
        else:
            self.overtake_direction = -1
            selected_side = 'derecha'

        self.preparing_overtake = True

        self.get_logger().debug(
            f'Preparando adelantamiento por la '
            f'{selected_side} | '
            f'oponente a {self.relative_opp_x:.2f} m'
        )
        
    def find_overtake_point(
        self,
        free_space,
        scan_msg
    ):
        center = len(free_space) // 2
        angle_increment = abs(scan_msg.angle_increment)

        # No buscamos en todo el costado.
        # Buscar entre 6° y 30° para hacer un cambio lateral moderado.
        inner_offset = max(
            2,
            int(math.radians(6.0) / angle_increment)
        )

        outer_offset = max(
            inner_offset + 1,
            int(math.radians(30.0) / angle_increment)
        )

        if self.overtake_direction > 0:
            # Izquierda
            region_start = min(
                len(free_space),
                center + inner_offset
            )

            region_end = min(
                len(free_space),
                center + outer_offset
            )
        else:
            # Derecha
            region_start = max(
                0,
                center - outer_offset
            )

            region_end = max(
                0,
                center - inner_offset
            )

        if region_end <= region_start:
            return None

        side_free_space = free_space[
            region_start:region_end
        ]

        local_gap_start, local_gap_end = (
            self.find_max_gap(side_free_space)
        )

        if local_gap_end <= local_gap_start:
            return None

        gap_start = region_start + local_gap_start
        gap_end = region_start + local_gap_end

        return self.find_best_point(
            free_space,
            gap_start,
            gap_end
        )
        
    def calculate_raw_steering_angle(
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

        return float(
            np.clip(
                steering_angle,
                -self.max_steering_angle,
                self.max_steering_angle
            )
        )
        
    def calculate_steering_angle(self, best_point, start_index, scan_msg):
        steering_angle = self.calculate_raw_steering_angle(
            best_point,
            start_index,
            scan_msg
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
            self.get_logger().debug(
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
        
        # Guardar posición del vehículo principal
        self.ego_x = x
        self.ego_y = y

        # Convertir la orientación quaternion a ángulo yaw
        orientation = odom_msg.pose.pose.orientation

        self.ego_yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y ** 2
                + orientation.z ** 2
            )
        )

        self.ego_speed = odom_msg.twist.twist.linear.x

        self.update_relative_opponent() 
     
    def opp_odom_callback(self, odom_msg):
        self.opp_x = odom_msg.pose.pose.position.x
        self.opp_y = odom_msg.pose.pose.position.y
        self.opp_speed = odom_msg.twist.twist.linear.x

        self.update_relative_opponent()

    def update_relative_opponent(self):
        # Esperar hasta tener la posición de ambos vehículos
        if (
            self.ego_x is None
            or self.ego_y is None
            or self.ego_yaw is None
            or self.opp_x is None
            or self.opp_y is None
        ):
            return

        # Diferencia entre las posiciones globales
        dx = self.opp_x - self.ego_x
        dy = self.opp_y - self.ego_y

        cos_yaw = math.cos(self.ego_yaw)
        sin_yaw = math.sin(self.ego_yaw)

        # Transformar la posición global al sistema del auto principal
        self.relative_opp_x = (
            cos_yaw * dx
            + sin_yaw * dy
        )

        self.relative_opp_y = (
            -sin_yaw * dx
            + cos_yaw * dy
        )

        current_time = self.get_clock().now().nanoseconds / 1e9

        # Mostrar información una vez por segundo
        if current_time - self.last_opponent_log_time >= 1.0:
            self.get_logger().debug(
                f'Oponente relativo | '
                f'frontal: {self.relative_opp_x:.2f} m | '
                f'lateral: {self.relative_opp_y:.2f} m | '
                f'velocidad: {self.opp_speed:.2f} m/s'
            )

            self.last_opponent_log_time = current_time
        
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

        # Primero calculamos la trayectoria normal del Follow the Gap.
        normal_gap_start, normal_gap_end = (
            self.find_max_gap(free_space)
        )

        normal_best_point = self.find_best_point(
            free_space,
            normal_gap_start,
            normal_gap_end
        )

        if normal_best_point is None:
            self.publish_drive(0.0, 0.0)
            return

        # Calcular el giro que usaría normalmente Follow the Gap.
        # Solo se utiliza para saber si estamos en recta o curva.
        normal_steering = (
            self.calculate_raw_steering_angle(
                normal_best_point,
                start_index,
                scan_msg
            )
        )

        self.update_overtake_state(
            front_ranges,
            scan_msg
        )

        best_point = normal_best_point

        if self.preparing_overtake or self.overtaking:
            overtake_point = self.find_overtake_point(
                free_space,
                scan_msg
            )

            if overtake_point is not None:
                if self.overtaking:
                    # Movimiento lateral más fuerte durante el rebase.
                    if abs(normal_steering) < 0.06:
                        overtake_weight = 0.68
                    else:
                        overtake_weight = 0.58
                else:
                    # Movimiento lateral suave desde aproximadamente 9 m.
                    if abs(normal_steering) < 0.06:
                        overtake_weight = 0.32
                    else:
                        overtake_weight = 0.22

                best_point = int(
                    overtake_weight * overtake_point
                    + (1.0 - overtake_weight)
                    * normal_best_point
                )

        steering_angle = (
            self.calculate_steering_angle(
                best_point,
                start_index,
                scan_msg
            )
        )

        speed = self.calculate_speed(
            steering_angle,
            front_ranges
        )

        # Durante la preparación se reduce un poco la aproximación,
        # mientras el vehículo comienza a colocarse hacia un costado.
        if self.preparing_overtake:
            speed = min(
                speed,
                self.pre_overtake_speed_limit
            )

        # Durante el adelantamiento completo se mantiene una
        # velocidad superior a la del oponente.
        if self.overtaking:
            left_clearance, right_clearance = (
                self.get_side_clearance(
                    front_ranges,
                    scan_msg
                )
            )

            if self.overtake_direction > 0:
                selected_clearance = left_clearance
            else:
                selected_clearance = right_clearance

            if selected_clearance >= 0.75:
                speed = max(
                    speed,
                    self.overtake_min_speed
                )

            speed = min(
                speed,
                self.overtake_speed_limit
            )

        self.publish_drive(
            speed,
            steering_angle
        )
        
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
