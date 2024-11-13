#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import serial
import struct
import math
import platform
import serial.tools.list_ports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from tf_transformations import quaternion_from_euler

# Function to find ttyUSB devices
def find_ttyUSB():
    print('Default IMU serial port is /dev/ttyUSB0. If multiple serial devices are recognized, please modify the corresponding port in the launch file.')
    ports = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    print('Current computer has {} serial devices: {}'.format(len(ports), ports))

# Checksum function
def checkSum(list_data, check_data):
    return sum(list_data) & 0xFF == check_data

# Convert 16-bit hex to short
def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # Parâmetros
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 921600)
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Publicadores
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/mag', 10)

        # Timer para publicar a 5 Hz
        self.timer = self.create_timer(0.2, self.publish_imu_data)  # 5 Hz

        # Buffers e variáveis
        self.buff = {}
        self.key = 0
        self.angularVelocity = [0.0, 0.0, 0.0]
        self.acceleration = [0.0, 0.0, 0.0]
        self.magnetometer = [0.0, 0.0, 0.0]
        self.angle_degree = [0.0, 0.0, 0.0]
        self.pub_flag = [True, True, True, True]

        self.python_version = platform.python_version()[0]

        find_ttyUSB()

        try:
            self.hf_imu = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0.5)
            if self.hf_imu.isOpen():
                self.get_logger().info('Serial port opened successfully.')
            else:
                self.hf_imu.open()
                self.get_logger().info('Serial port opened successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            exit(0)

        # Timer para ler dados de serial a 200 Hz
        self.read_timer = self.create_timer(0.005, self.read_serial_data)  # 200 Hz

    def read_serial_data(self):
        try:
            buff_count = self.hf_imu.in_waiting
        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
            self.get_logger().error('IMU disconnected or connection issue.')
            exit(0)
        else:
            if buff_count > 0:
                buff_data = self.hf_imu.read(buff_count)
                for i in range(0, buff_count):
                    self.handle_serial_data(buff_data[i])

    def handle_serial_data(self, raw_data):
        if self.python_version == '2':
            raw_data = ord(raw_data)

        self.buff[self.key] = raw_data
        self.key += 1

        if self.buff.get(0, None) != 0x55:
            self.key = 0
            self.buff = {}
            return

        if self.key < 11:
            return
        else:
            data_buff = list(self.buff.values())

            if self.buff[1] == 0x51 and self.pub_flag[0]:
                if checkSum(data_buff[0:10], data_buff[10]):
                    self.acceleration = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(3)]
                else:
                    self.get_logger().warn('0x51 checksum failed.')
                self.pub_flag[0] = False

            elif self.buff[1] == 0x52 and self.pub_flag[1]:
                if checkSum(data_buff[0:10], data_buff[10]):
                    self.angularVelocity = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in range(3)]
                else:
                    self.get_logger().warn('0x52 checksum failed.')
                self.pub_flag[1] = False

            elif self.buff[1] == 0x53 and self.pub_flag[2]:
                if checkSum(data_buff[0:10], data_buff[10]):
                    self.angle_degree = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(3)]
                else:
                    self.get_logger().warn('0x53 checksum failed.')
                self.pub_flag[2] = False

            elif self.buff[1] == 0x54 and self.pub_flag[3]:
                if checkSum(data_buff[0:10], data_buff[10]):
                    magnetometer_raw = hex_to_short(data_buff[2:10])
                    self.magnetometer = [float(magnetometer_raw[i]) for i in range(3)]
                else:
                    self.get_logger().warn('0x54 checksum failed.')
                self.pub_flag[3] = False

            else:
                self.get_logger().warn(f'No parsing provided for data type: {self.buff[1]}')
                self.buff = {}
                self.key = 0
                return

            self.buff = {}
            self.key = 0

            if all(not flag for flag in self.pub_flag):
                # Dados completos recebidos, armazenar para publicação
                self.imu_data = {
                    'angular_velocity': self.angularVelocity,
                    'linear_acceleration': self.acceleration,
                    'orientation_deg': self.angle_degree,
                    'magnetic_field': self.magnetometer
                }

    def publish_imu_data(self):
        if hasattr(self, 'imu_data'):
            imu_msg = Imu()
            mag_msg = MagneticField()
            current_time = self.get_clock().now().to_msg()

            imu_msg.header.stamp = current_time
            imu_msg.header.frame_id = 'base_footprint'

            mag_msg.header.stamp = current_time
            mag_msg.header.frame_id = 'base_footprint'

            # Converter RPY para Quaternion
            angle_radian = [angle * math.pi / 180 for angle in self.imu_data['orientation_deg']]
            qua = quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])

            imu_msg.orientation.x = float(qua[0])
            imu_msg.orientation.y = float(qua[1])
            imu_msg.orientation.z = float(qua[2])
            imu_msg.orientation.w = float(qua[3])

            imu_msg.angular_velocity.x = float(self.imu_data['angular_velocity'][0])
            imu_msg.angular_velocity.y = float(self.imu_data['angular_velocity'][1])
            imu_msg.angular_velocity.z = float(self.imu_data['angular_velocity'][2])

            imu_msg.linear_acceleration.x = float(self.imu_data['linear_acceleration'][0])
            imu_msg.linear_acceleration.y = float(self.imu_data['linear_acceleration'][1])
            imu_msg.linear_acceleration.z = float(self.imu_data['linear_acceleration'][2])

            mag_msg.magnetic_field.x = float(self.imu_data['magnetic_field'][0])
            mag_msg.magnetic_field.y = float(self.imu_data['magnetic_field'][1])
            mag_msg.magnetic_field.z = float(self.imu_data['magnetic_field'][2])

            self.imu_pub.publish(imu_msg)
            self.mag_pub.publish(mag_msg)

            self.get_logger().debug(f'Publicou /imu com Yaw: {self.imu_data["orientation_deg"][2]:.2f} graus')

            del self.imu_data  # Limpar os dados após publicação

def main(args=None):
    rclpy.init(args=args)
    imu_node = ImuNode()
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        imu_node.get_logger().info('Finalizando nó ImuNode.')
    finally:
        imu_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
