import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import json
from geometry_msgs.msg import Quaternion
import math

def euler_to_quaternion(roll, pitch, yaw):
    # Konversi dari Euler (rad) ke Quaternion
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q

class IMUNode(Node):
    def __init__(self):
        super().__init__('serial_imu_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Ganti sesuai port IMU Razor kamu
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

    def timer_callback(self):
        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            data = json.loads(line)

            imu_msg = Imu()

            # Header untuk sinkronisasi RViz
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'base_link'

            # Ambil data Euler (dalam derajat), konversi ke radian
            roll = math.radians(data['roll'])
            pitch = math.radians(data['pitch'])
            yaw = math.radians(data['yaw'])

            # Konversi ke Quaternion
            quat = euler_to_quaternion(roll, pitch, yaw)
            imu_msg.orientation = quat

            # Orientation covariance (diisi float, harus 9 elemen)
            imu_msg.orientation_covariance = [0.02, 0.0, 0.0,
                                              0.0, 0.02, 0.0,
                                              0.0, 0.0, 0.02]

            # Akselerasi (dalam m/s²)
            imu_msg.linear_acceleration.x = float(data['accX'])
            imu_msg.linear_acceleration.y = float(data['accY'])
            imu_msg.linear_acceleration.z = float(data['accZ'])
            imu_msg.linear_acceleration_covariance = [0.1, 0.0, 0.0,
                                                      0.0, 0.1, 0.0,
                                                      0.0, 0.0, 0.1]

            # Angular velocity kosong (jika IMU Razor tidak mengirim)
            imu_msg.angular_velocity.x = 0.0
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.z = 0.0
            imu_msg.angular_velocity_covariance = [0.05, 0.0, 0.0,
                                                   0.0, 0.05, 0.0,
                                                   0.0, 0.0, 0.05]

            # Publikasi ke ROS
            self.publisher_.publish(imu_msg)

        except (json.JSONDecodeError, KeyError, ValueError):
            self.get_logger().warn("Data JSON tidak valid atau incomplete.")
        except serial.SerialException:
            self.get_logger().error("Gagal membaca dari serial port.")

def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
