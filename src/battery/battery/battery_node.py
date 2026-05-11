#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from typing import Optional

import requests
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node')

        # =========================
        # 사용자 설정
        # =========================
        self.server_base_url = 'http://192.168.0.221:8000'

        self.serial_port = '/dev/ttyUSB2'
        self.baudrate = 9600
        self.pdist_id = 1

        self.battery_topic = '/robot/battery'
        self.server_endpoint = '/robot/battery'

        self.update_period_sec = 60.0
        self.request_timeout_sec = 5.0

        # 24V 배터리 전압 기반 퍼센트 추정 기준
        # PDIST120은 SOC 직접값이 아니라 입력 전압을 읽으므로 간접 추정값입니다.
        self.empty_voltage = 21.5
        self.full_voltage = 29.4

        # =========================
        # ROS parameter
        # =========================
        self.declare_parameter('server_base_url', self.server_base_url)
        self.declare_parameter('serial_port', self.serial_port)
        self.declare_parameter('baudrate', self.baudrate)
        self.declare_parameter('pdist_id', self.pdist_id)

        self.declare_parameter('battery_topic', self.battery_topic)
        self.declare_parameter('server_endpoint', self.server_endpoint)

        self.declare_parameter('update_period_sec', self.update_period_sec)
        self.declare_parameter('request_timeout_sec', self.request_timeout_sec)

        self.declare_parameter('empty_voltage', self.empty_voltage)
        self.declare_parameter('full_voltage', self.full_voltage)

        self.declare_parameter('enable_server_post', True)
        self.declare_parameter('enable_topic_publish', True)

        self.server_base_url = self.get_parameter('server_base_url').value.rstrip('/')
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.pdist_id = int(self.get_parameter('pdist_id').value)

        self.battery_topic = self.get_parameter('battery_topic').value
        self.server_endpoint = self.get_parameter('server_endpoint').value

        self.update_period_sec = float(self.get_parameter('update_period_sec').value)
        self.request_timeout_sec = float(self.get_parameter('request_timeout_sec').value)

        self.empty_voltage = float(self.get_parameter('empty_voltage').value)
        self.full_voltage = float(self.get_parameter('full_voltage').value)

        self.enable_server_post = bool(self.get_parameter('enable_server_post').value)
        self.enable_topic_publish = bool(self.get_parameter('enable_topic_publish').value)

        # =========================
        # PDIST packet constants
        # =========================
        self.MID_PDIST = 186          # 0xBA
        self.TMID = 172               # 0xAC
        self.PID_REQ_PID_DATA = 4     # 0x04
        self.PID_VOLT_IN = 143        # 0x8F

        # =========================
        # 내부 상태
        # =========================
        self.http = requests.Session()
        self.ser: Optional[serial.Serial] = None

        # =========================
        # ROS interface
        # =========================
        self.battery_pub = self.create_publisher(Int32, self.battery_topic, 10)

        self.open_serial()

        # 시작 후 5초 뒤 1회 즉시 갱신
        self.first_update_timer = self.create_timer(5.0, self.first_update_once)

        # 이후 60초마다 갱신
        self.timer = self.create_timer(self.update_period_sec, self.update_loop)

        self.get_logger().info(
            f'BatteryNode started | '
            f'server={self.server_base_url} | '
            f'endpoint={self.server_endpoint} | '
            f'port={self.serial_port} | '
            f'topic={self.battery_topic} | '
            f'period={self.update_period_sec:.1f}s'
        )

    # =========================
    # Serial
    # =========================
    def open_serial(self):
        try:
            if self.ser is not None and self.ser.is_open:
                return

            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0,
            )

            self.get_logger().info(f'serial opened: {self.serial_port}')

        except Exception as e:
            self.ser = None
            self.get_logger().error(f'serial open failed: {self.serial_port} | {e}')

    def close_serial(self):
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

        self.ser = None

    # =========================
    # PDIST packet
    # =========================
    @staticmethod
    def checksum(data: bytes) -> int:
        return (-sum(data)) & 0xFF

    @staticmethod
    def valid_checksum(packet: bytes) -> bool:
        return (sum(packet) & 0xFF) == 0

    def build_voltage_request(self) -> bytes:
        body = bytes([
            self.MID_PDIST,
            self.TMID,
            self.pdist_id,
            self.PID_REQ_PID_DATA,
            1,
            self.PID_VOLT_IN,
        ])

        return body + bytes([self.checksum(body)])

    def read_voltage_once(self) -> float:
        if self.ser is None or not self.ser.is_open:
            self.open_serial()

        if self.ser is None or not self.ser.is_open:
            raise RuntimeError('serial port is not open')

        req = self.build_voltage_request()

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.ser.write(req)
        self.ser.flush()

        # 예상 응답:
        # [TMID, MID, ID, PID, DataNumber, D1, D2, CHK]
        resp = self.ser.read(8)

        if len(resp) != 8:
            raise RuntimeError(
                f'PDIST response length error: {len(resp)} bytes, raw={resp.hex(" ")}'
            )

        if not self.valid_checksum(resp):
            raise RuntimeError(f'PDIST checksum error: raw={resp.hex(" ")}')

        rx_tmid = resp[0]
        rx_mid = resp[1]
        rx_id = resp[2]
        rx_pid = resp[3]
        data_num = resp[4]
        d1 = resp[5]
        d2 = resp[6]

        if rx_tmid != self.TMID:
            raise RuntimeError(f'TMID mismatch: {rx_tmid}, raw={resp.hex(" ")}')

        # 매뉴얼 표기 차이 대응: 183 또는 186 허용
        if rx_mid not in (183, 186):
            raise RuntimeError(f'MID mismatch: {rx_mid}, raw={resp.hex(" ")}')

        if rx_id != self.pdist_id:
            raise RuntimeError(f'PDIST ID mismatch: {rx_id}, raw={resp.hex(" ")}')

        if rx_pid != self.PID_VOLT_IN:
            raise RuntimeError(f'PID mismatch: {rx_pid}, raw={resp.hex(" ")}')

        if data_num != 2:
            raise RuntimeError(f'DataNumber mismatch: {data_num}, raw={resp.hex(" ")}')

        raw_voltage = d1 + (d2 << 8)
        voltage = raw_voltage * 0.1

        return voltage

    # =========================
    # Battery percent
    # =========================
    def voltage_to_percentage(self, voltage: float) -> int:
        if self.full_voltage <= self.empty_voltage:
            self.get_logger().warn('full_voltage must be greater than empty_voltage. return 0%.')
            return 0

        percentage = (voltage - self.empty_voltage) / (self.full_voltage - self.empty_voltage) * 100.0
        percentage = max(0.0, min(100.0, percentage))

        return int(round(percentage))

    # =========================
    # ROS publish
    # =========================
    def publish_battery(self, percentage: int):
        if not self.enable_topic_publish:
            return

        msg = Int32()
        msg.data = int(percentage)
        self.battery_pub.publish(msg)

    # =========================
    # Server post
    # =========================
    def send_battery(self, percentage: int):
        if not self.enable_server_post:
            return

        payload = {
            'percentage': int(percentage)
        }

        try:
            resp = self.http.post(
                f'{self.server_base_url}{self.server_endpoint}',
                json=payload,
                timeout=self.request_timeout_sec
            )
            resp.raise_for_status()

            self.get_logger().info(
                f'/robot/battery sent | payload={payload} | status={resp.status_code}'
            )

        except Exception as e:
            self.get_logger().error(f'/robot/battery failed: {e}')

    # =========================
    # Main loop
    # =========================
    def first_update_once(self):
        try:
            self.first_update_timer.cancel()
        except Exception:
            pass

        self.update_loop()

    def update_loop(self):
        try:
            voltage = self.read_voltage_once()
            percentage = self.voltage_to_percentage(voltage)

            self.publish_battery(percentage)
            self.send_battery(percentage)

            self.get_logger().info(
                f'battery update | voltage={voltage:.1f}V | percentage={percentage}%'
            )

        except Exception as e:
            self.get_logger().error(f'battery update failed: {e}')

            # 다음 주기에서 재연결 시도
            self.close_serial()

    def destroy_node(self):
        self.close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 안전하게 노드를 종료합니다.')
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
