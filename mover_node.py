import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import tf2_ros
import math


class RectangleRobot(Node):

    def __init__(self):
        super().__init__('rectangle_robot')

        # ==========================================
        # Publisher CMD_VEL
        # ==========================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # ==========================================
        # TF BUFFER
        # ==========================================

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # ==========================================
        # TIMER
        # ==========================================

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        # ==========================================
        # KONFIGURASI PERSEGI PANJANG
        # ==========================================

        self.length = 2.0      # meter
        self.width = 1.0       # meter

        # Kecepatan
        self.linear_speed = 0.4
        self.angular_speed = 0.3

        # ==========================================
        # STATE
        # ==========================================

        self.state = 'FORWARD'

        self.side_count = 0

        self.start_x = None
        self.start_y = None

        self.start_yaw = None
        self.target_yaw = None

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.get_logger().info(
            '===================================='
        )

        self.get_logger().info(
            'Rectangle Robot Started'
        )

        self.get_logger().info(
            'Target: 2 m x 1 m'
        )

        self.get_logger().info(
            '===================================='
        )

    # ==================================================
    # GET ROBOT POSE DARI TF
    # ==================================================

    def get_robot_pose(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                'odom',
                'base_footprint',
                rclpy.time.Time()
            )

        except Exception:

            return None

        # ------------------------------------------
        # POSITION
        # ------------------------------------------

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        # ------------------------------------------
        # QUATERNION
        # ------------------------------------------

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        # ------------------------------------------
        # QUATERNION -> YAW
        # ------------------------------------------

        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

        return x, y, yaw

    # ==================================================
    # NORMALISASI ANGLE
    # ==================================================

    def normalize_angle(self, angle):

        while angle > math.pi:

            angle -= 2.0 * math.pi

        while angle < -math.pi:

            angle += 2.0 * math.pi

        return angle

    # ==================================================
    # CONTROL LOOP
    # ==================================================

    def control_loop(self):

        msg = Twist()

        pose = self.get_robot_pose()

        # ==========================================
        # TF BELUM TERSEDIA
        # ==========================================

        if pose is None:

            self.get_logger().warn(
                'Menunggu TF odom -> base_footprint...'
            )

            return

        # ==========================================
        # UPDATE POSE
        # ==========================================

        self.current_x = pose[0]
        self.current_y = pose[1]
        self.current_yaw = pose[2]

        # ==========================================
        # STATE: FORWARD
        # ==========================================

        if self.state == 'FORWARD':

            # --------------------------------------
            # Jika baru mulai sisi
            # --------------------------------------

            if self.start_x is None:

                self.start_x = self.current_x
                self.start_y = self.current_y

                if self.side_count % 2 == 0:

                    target_distance = self.length

                else:

                    target_distance = self.width

                self.current_target_distance = target_distance

                self.get_logger().info(
                    f'Mulai sisi {self.side_count + 1} '
                    f'| Target = '
                    f'{target_distance:.2f} m'
                )

            # --------------------------------------
            # HITUNG JARAK
            # --------------------------------------

            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y

            distance = math.sqrt(
                dx * dx + dy * dy
            )

            # --------------------------------------
            # BELUM SAMPAI TARGET
            # --------------------------------------

            if distance < self.current_target_distance:

                msg.linear.x = self.linear_speed
                msg.angular.z = 0.0

                self.get_logger().info(
                    f'Maju | '
                    f'Jarak = {distance:.2f} / '
                    f'{self.current_target_distance:.2f} m'
                )

            # --------------------------------------
            # SUDAH SAMPAI
            # --------------------------------------

            else:

                msg.linear.x = 0.0
                msg.angular.z = 0.0

                self.get_logger().info(
                    f'Sisi {self.side_count + 1} selesai '
                    f'| Jarak = {distance:.2f} m'
                )

                # Reset posisi awal
                self.start_x = None
                self.start_y = None

                # Ambil yaw sekarang
                self.start_yaw = self.current_yaw

                # Target CCW +90°
                self.target_yaw = self.normalize_angle(
                    self.start_yaw + math.pi / 2.0
                )

                self.state = 'ROTATE'

        # ==========================================
        # STATE: ROTATE
        # ==========================================

        elif self.state == 'ROTATE':

            error = self.normalize_angle(
                self.target_yaw - self.current_yaw
            )

            error_deg = math.degrees(error)

            # --------------------------------------
            # BELUM 90°
            # --------------------------------------

            if abs(error) > math.radians(2.0):

                msg.linear.x = 0.0

                # CCW
                if error > 0:

                    msg.angular.z = self.angular_speed

                # CW
                else:

                    msg.angular.z = -self.angular_speed

                self.get_logger().info(
                    f'Rotasi | '
                    f'Yaw = '
                    f'{math.degrees(self.current_yaw):.1f}° | '
                    f'Target = '
                    f'{math.degrees(self.target_yaw):.1f}° | '
                    f'Error = {error_deg:.1f}°'
                )

            # --------------------------------------
            # SUDAH 90°
            # --------------------------------------

            else:

                msg.linear.x = 0.0
                msg.angular.z = 0.0

                self.get_logger().info(
                    f'Rotasi 90° selesai | '
                    f'Yaw = '
                    f'{math.degrees(self.current_yaw):.2f}°'
                )

                self.side_count += 1

                # ----------------------------------
                # SUDAH 4 SISI
                # ----------------------------------

                if self.side_count >= 4:

                    self.state = 'STOP'

                else:

                    self.state = 'FORWARD'

        # ==========================================
        # STATE: STOP
        # ==========================================

        elif self.state == 'STOP':

            msg.linear.x = 0.0
            msg.angular.z = 0.0

            self.cmd_pub.publish(msg)

            self.get_logger().info(
                '===================================='
            )

            self.get_logger().info(
                'PERSEGI PANJANG SELESAI'
            )

            self.get_logger().info(
                '===================================='
            )

            self.timer.cancel()

            return

        # ==========================================
        # PUBLISH
        # ==========================================

        self.cmd_pub.publish(msg)


# ==================================================
# MAIN
# ==================================================

def main(args=None):

    rclpy.init(args=args)

    node = RectangleRobot()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        # Kirim STOP
        msg = Twist()

        msg.linear.x = 0.0
        msg.angular.z = 0.0

        node.cmd_pub.publish(msg)

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()