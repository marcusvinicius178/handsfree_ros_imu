#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import serial
import struct
import math
import platform
import serial.tools.list_ports
import time

# Function to find ttyUSB devices
def find_ttyUSB():
    print('Default IMU serial port is /dev/ttyUSB0. If multiple serial devices are recognized, please modify the port accordingly.')
    ports = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    print('Current computer has {} serial devices: {}'.format(len(ports), ports))

# Checksum function
def checkSum(list_data, check_data):
    return sum(list_data) & 0xFF == check_data

# Convert 16-bit hex to short
def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))

class IMUReader:
    def __init__(self, port='/dev/ttyUSB0', baudrate=921600):
        self.port = port
        self.baudrate = baudrate

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
                print('Serial port opened successfully.')
            else:
                self.hf_imu.open()
                print('Serial port opened successfully.')
        except Exception as e:
            print('Failed to open serial port: {}'.format(e))
            exit(0)

    def read_serial_data(self):
        try:
            buff_count = self.hf_imu.in_waiting
        except Exception as e:
            print('Exception:', e)
            print('IMU disconnected or connection issue.')
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

        if self.buff[0] != 0x55:
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
                    print('0x51 checksum failed.')
                self.pub_flag[0] = False

            elif self.buff[1] == 0x52 and self.pub_flag[1]:
                if checkSum(data_buff[0:10], data_buff[10]):
                    self.angularVelocity = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in range(3)]
                else:
                    print('0x52 checksum failed.')
                self.pub_flag[1] = False

            elif self.buff[1] == 0x53 and self.pub_flag[2]:
                if checkSum(data_buff[0:10], data_buff[10]):
                    self.angle_degree = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(3)]
                else:
                    print('0x53 checksum failed.')
                self.pub_flag[2] = False

            elif self.buff[1] == 0x54 and self.pub_flag[3]:
                if checkSum(data_buff[0:10], data_buff[10]):
                    self.magnetometer = hex_to_short(data_buff[2:10])
                else:
                    print('0x54 checksum failed.')
                self.pub_flag[3] = False

            else:
                print('No parsing provided for data type: {}'.format(self.buff[1]))
                self.buff = {}
                self.key = 0
                return

            self.buff = {}
            self.key = 0

            if all(not flag for flag in self.pub_flag):
                self.print_imu_data()
                self.pub_flag = [True, True, True, True]

    def print_imu_data(self):
        print('Acceleration (m/s^2):', self.acceleration)
        print('Angular Velocity (rad/s):', self.angularVelocity)
        print('Angle (degrees):', self.angle_degree)
        print('Magnetometer:', self.magnetometer)
        print('-' * 50)

def main():
    imu_reader = IMUReader(port='/dev/ttyUSB0', baudrate=921600)
    try:
        while True:
            imu_reader.read_serial_data()
            time.sleep(0.005)  # Sleep for 5ms (200Hz)
    except KeyboardInterrupt:
        print('Exiting...')

if __name__ == '__main__':
    main()
