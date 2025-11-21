#!/usr/bin/env python3
import rclpy, socket, threading, time
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState

def crc8_maxim(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8C) if (crc & 0x01) else (crc >> 1)
    return crc & 0xFF

class WifiSerialBridgeUDP(Node):
    """
    UDP bridge:
      - En ROLL: envía CMD_ROLL (1) con v_left (v1) y v_right (v2) a ESP32.
      - WALK: anuncio (2), START (6), STOP (0).
      - E-STOP (3)/RESET (4).
      - Escucha respuestas UDP en local_port y las loguea como [RX].
    """
    def __init__(self):
        super().__init__('wifi_serial_bridge')
        p = self.declare_parameter
        self.esp_host = p('esp_host', '192.168.137.245').value
        self.esp_port = p('esp_port', 9000).value
        self.local_port = p('local_port', 9001).value

        self.vel_rate_hz = p('vel_rate_hz', 10.0).value

        self.body_id_hex = p('body_id_hex', '0x000').value
        self.CMD_IDLE        = p('cmd_idle', 0).value
        self.CMD_ROLL        = p('cmd_roll', 1).value
        self.CMD_WALK        = p('cmd_walk', 2).value
        self.CMD_ESTOP       = p('cmd_estop', 3).value
        self.CMD_ERES        = p('cmd_ereset', 4).value
        self.CMD_WALK_START  = p('cmd_walk_start', 6).value

        self.mode = 'ROLL'
        self.v_left = 0.0
        self.v_right = 0.0

        # UDP socket (envío)
        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx_addr = (self.esp_host, self.esp_port)

        # UDP socket (recepción)
        self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock.bind(('0.0.0.0', self.local_port))
        self.rx_sock.settimeout(0.2)
        self.reader_stop = threading.Event()
        threading.Thread(target=self._reader_loop, daemon=True).start()

        self.create_subscription(JointState, '/joint_states', self.on_wheels, 50)
        self.create_subscription(String, '/gait_cmd', self.on_gait, 10)
        self.create_subscription(String, '/mode_cmd', self.on_mode, 10)
        self.create_subscription(Bool,   '/estop', self.on_estop, 10)
        self.create_subscription(Bool,   '/estop_reset', self.on_estop_reset, 10)

        self.create_timer(1.0/self.vel_rate_hz, self.tick_roll)

        self.get_logger().info(f'UDP bridge → ESP32 {self.esp_host}:{self.esp_port} (listen {self.local_port})')

    def _reader_loop(self):
        while not self.reader_stop.is_set():
            try:
                data, addr = self.rx_sock.recvfrom(1024)
                if not data: continue
                s = data.decode('utf-8', errors='replace').rstrip('\r\n')
                self.get_logger().info(f'[RX] {s}')
            except socket.timeout:
                pass
            except Exception as e:
                self.get_logger().warn(f'RX UDP error: {e}')
                time.sleep(0.1)

    def _send_frame_ascii(self, v1, v2, v3, cmd, id_hex):
        payload = f"{v1:.3f},{v2:.3f},{v3:.3f},{int(cmd)},{id_hex}"
        crc = crc8_maxim(payload.encode('ascii'))
        frame = f"<{payload},{crc:02X}>\n"
        try:
            self.tx_sock.sendto(frame.encode('ascii'), self.tx_addr)
            self.get_logger().info(f'[TX] {frame.strip()}')
        except Exception as e:
            self.get_logger().warn(f'TX UDP error: {e}')

    # ---- Callbacks ----
    def on_wheels(self, msg):
        self.v_left = msg.velocity[0]
        self.v_right = msg.velocity[1]

    def on_gait(self, msg: String):
        m = msg.data.upper()
        if m == 'START':
            self._send_frame_ascii(0.0, 0.0, 0.0, self.CMD_WALK_START, self.body_id_hex)
        elif m == 'STOP':
            self._send_frame_ascii(0.0, 0.0, 0.0, self.CMD_IDLE, self.body_id_hex)

    def on_mode(self, msg: String):
        m = msg.data.upper()
        if m in ('ROLL','WALK'):
            self.mode = m
            cmd = self.CMD_ROLL if m == 'ROLL' else self.CMD_WALK
            self._send_frame_ascii(0.0, 0.0, 0.0, cmd, self.body_id_hex)

    def on_estop(self, msg: Bool):
        if msg.data:
            self._send_frame_ascii(0.0, 0.0, 0.0, self.CMD_ESTOP, self.body_id_hex)

    def on_estop_reset(self, msg: Bool):
        if msg.data:
            self._send_frame_ascii(0.0, 0.0, 0.0, self.CMD_ERES, self.body_id_hex)

    def tick_roll(self):
        if self.mode == 'ROLL':
            self._send_frame_ascii(self.v_left, self.v_right, 0.0, self.CMD_ROLL, self.body_id_hex)

def main():
    rclpy.init()
    node = WifiSerialBridgeUDP()
    try:
        rclpy.spin(node)
    finally:
        node.reader_stop.set()
        node.rx_sock.close()
        node.tx_sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
