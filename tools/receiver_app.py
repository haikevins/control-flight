#!/usr/bin/env python3
import argparse
import queue
import struct
import threading
import time
from collections import deque

import serial
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg

START_BYTE = 0xAA
MAX_PAYLOAD = 32

PKT_HEARTBEAT = 1
PKT_COMMAND = 2
PKT_IMU = 3
PKT_THROTTLE = 4
PKT_LINK = 5

IMU_STRUCT = struct.Struct("<Hhhhhhhhhh")
THROTTLE_STRUCT = struct.Struct("<HHHH")
LINK_STRUCT = struct.Struct("<bHH")


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) & 0xFFFF) ^ 0x1021
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class SerialManager:
    def __init__(self, port: str, baud: int):
        self.serial = serial.Serial(port, baudrate=baud, timeout=0.05)
        self.lock = threading.Lock()

    def read_bytes(self, size: int = 256) -> bytes:
        return self.serial.read(size)

    def write_frame(self, pkt_type: int, payload: bytes) -> None:
        length = len(payload)
        frame = bytearray()
        frame.append(START_BYTE)
        frame.append(pkt_type & 0xFF)
        frame.extend(struct.pack("<H", length))
        frame.extend(payload)
        frame.extend(struct.pack("<H", crc16_ccitt(payload)))
        with self.lock:
            self.serial.write(frame)

    def close(self) -> None:
        with self.lock:
            self.serial.close()


class SerialReaderThread(threading.Thread):
    def __init__(self, serial_mgr: SerialManager, byte_queue: queue.Queue, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.serial_mgr = serial_mgr
        self.byte_queue = byte_queue
        self.stop_event = stop_event

    def run(self) -> None:
        while not self.stop_event.is_set():
            data = self.serial_mgr.read_bytes()
            if not data:
                continue
            for byte in data:
                self.byte_queue.put(byte)


class ParserThread(threading.Thread):
    def __init__(self, byte_queue: queue.Queue, packet_queue: queue.Queue, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.byte_queue = byte_queue
        self.packet_queue = packet_queue
        self.stop_event = stop_event
        self.state = "WAIT_START"
        self.pkt_type = 0
        self.length = 0
        self.payload = bytearray()
        self.crc = 0

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                byte = self.byte_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            self._consume_byte(byte)

    def _consume_byte(self, byte: int) -> None:
        if self.state == "WAIT_START":
            if byte == START_BYTE:
                self.state = "READ_TYPE"
        elif self.state == "READ_TYPE":
            self.pkt_type = byte
            self.state = "READ_LEN_1"
        elif self.state == "READ_LEN_1":
            self.length = byte
            self.state = "READ_LEN_2"
        elif self.state == "READ_LEN_2":
            self.length |= byte << 8
            if self.length > MAX_PAYLOAD:
                self.state = "WAIT_START"
            else:
                self.payload = bytearray()
                self.state = "READ_PAYLOAD" if self.length > 0 else "READ_CRC_1"
        elif self.state == "READ_PAYLOAD":
            self.payload.append(byte)
            if len(self.payload) >= self.length:
                self.state = "READ_CRC_1"
        elif self.state == "READ_CRC_1":
            self.crc = byte
            self.state = "READ_CRC_2"
        elif self.state == "READ_CRC_2":
            self.crc |= byte << 8
            if crc16_ccitt(self.payload) == self.crc:
                self.packet_queue.put((self.pkt_type, bytes(self.payload)))
            self.state = "WAIT_START"
        else:
            self.state = "WAIT_START"


class HeartbeatThread(threading.Thread):
    def __init__(self, serial_mgr: SerialManager, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.serial_mgr = serial_mgr
        self.stop_event = stop_event
        self.counter = 0

    def run(self) -> None:
        while not self.stop_event.is_set():
            payload = struct.pack("<I", self.counter & 0xFFFFFFFF)
            self.serial_mgr.write_frame(PKT_HEARTBEAT, payload)
            self.counter += 1
            time.sleep(0.05)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, serial_mgr: SerialManager, packet_queue: queue.Queue, stop_event: threading.Event):
        super().__init__()
        self.serial_mgr = serial_mgr
        self.packet_queue = packet_queue
        self.stop_event = stop_event
        self.streaming = False

        self.start_time = None
        self.last_rssi = 0
        self.last_loss = 0
        self.last_heartbeat = 0
        self.imu_packets = 0
        self.fps = 0.0
        self.last_fps_time = time.monotonic()

        self._init_ui()

        self.update_timer = QtCore.QTimer(self)
        self.update_timer.timeout.connect(self._process_packets)
        self.update_timer.start(50)

    def _init_ui(self) -> None:
        self.setWindowTitle("IMU Realtime Viewer")
        pg.setConfigOption("background", (25, 27, 30))
        pg.setConfigOption("foreground", (220, 220, 220))

        self.start_button = QtWidgets.QPushButton("START")
        self.stop_button = QtWidgets.QPushButton("STOP")
        self.stop_button.setEnabled(False)
        self.start_button.clicked.connect(self._start_stream)
        self.stop_button.clicked.connect(self._stop_stream)

        top_bar = QtWidgets.QHBoxLayout()
        top_bar.addWidget(self.start_button)
        top_bar.addWidget(self.stop_button)
        top_bar.addStretch()

        self.accel_plot, self.accel_curves = self._create_plot(
            "Accel (ax, ay, az)",
            "g",
            ["X", "Y", "Z"],
            [(255, 105, 180), (255, 215, 0), (0, 255, 255)],
        )
        self.gyro_plot, self.gyro_curves = self._create_plot(
            "Gyro (gx, gy, gz)",
            "deg/s",
            ["X", "Y", "Z"],
            [(255, 105, 180), (255, 215, 0), (0, 255, 255)],
        )
        self.angle_plot, self.angle_curves = self._create_plot(
            "Angles (roll, pitch, yaw)",
            "deg",
            ["Roll", "Pitch", "Yaw"],
            [(255, 105, 180), (255, 215, 0), (0, 255, 255)],
        )
        self.throttle_plot, self.throttle_curves = self._create_plot(
            "Throttle (m1, m2, m3, m4)",
            "pwm",
            ["M1", "M2", "M3", "M4"],
            [(255, 105, 180), (255, 215, 0), (0, 255, 255), (0, 200, 120)],
        )

        grid = QtWidgets.QGridLayout()
        grid.addWidget(self.accel_plot, 0, 0)
        grid.addWidget(self.gyro_plot, 0, 1)
        grid.addWidget(self.angle_plot, 1, 0)
        grid.addWidget(self.throttle_plot, 1, 1)

        container = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(container)
        layout.addLayout(top_bar)
        layout.addLayout(grid)
        self.setCentralWidget(container)

        self.status = QtWidgets.QStatusBar()
        self.setStatusBar(self.status)
        self._update_status_bar()

        self.samples = 300
        self.t_imu = deque(maxlen=self.samples)
        self.ax = deque(maxlen=self.samples)
        self.ay = deque(maxlen=self.samples)
        self.az = deque(maxlen=self.samples)
        self.gx = deque(maxlen=self.samples)
        self.gy = deque(maxlen=self.samples)
        self.gz = deque(maxlen=self.samples)
        self.roll = deque(maxlen=self.samples)
        self.pitch = deque(maxlen=self.samples)
        self.yaw = deque(maxlen=self.samples)

        self.t_thr = deque(maxlen=self.samples)
        self.m1 = deque(maxlen=self.samples)
        self.m2 = deque(maxlen=self.samples)
        self.m3 = deque(maxlen=self.samples)
        self.m4 = deque(maxlen=self.samples)

    def _create_plot(self, title: str, unit: str, labels, colors):
        plot = pg.PlotWidget()
        plot.setTitle(title)
        plot.setLabel("bottom", "time (s)")
        plot.setLabel("left", unit)
        plot.showGrid(x=True, y=True, alpha=0.3)
        plot.addLegend()
        curves = []
        for idx, label in enumerate(labels):
            color = colors[idx] if colors and idx < len(colors) else None
            curves.append(plot.plot(pen=pg.mkPen(color=color, width=1.5), name=label))
        return plot, curves

    def _start_stream(self) -> None:
        self.streaming = True
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)

    def _stop_stream(self) -> None:
        self.streaming = False
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)

    def _process_packets(self) -> None:
        updated_imu = False
        updated_thr = False
        status_dirty = False

        while True:
            try:
                pkt_type, payload = self.packet_queue.get_nowait()
            except queue.Empty:
                break

            now = time.monotonic()
            if self.start_time is None:
                self.start_time = now
            t = now - self.start_time

            if pkt_type == PKT_IMU and len(payload) >= IMU_STRUCT.size:
                imu = IMU_STRUCT.unpack(payload[:IMU_STRUCT.size])
                if self.streaming:
                    self.t_imu.append(t)
                    self.ax.append(imu[1])
                    self.ay.append(imu[2])
                    self.az.append(imu[3])
                    self.gx.append(imu[4])
                    self.gy.append(imu[5])
                    self.gz.append(imu[6])
                    self.roll.append(imu[7])
                    self.pitch.append(imu[8])
                    self.yaw.append(imu[9])
                    updated_imu = True
                self.imu_packets += 1
            elif pkt_type == PKT_THROTTLE and len(payload) >= THROTTLE_STRUCT.size:
                thr = THROTTLE_STRUCT.unpack(payload[:THROTTLE_STRUCT.size])
                if self.streaming:
                    self.t_thr.append(t)
                    self.m1.append(thr[0])
                    self.m2.append(thr[1])
                    self.m3.append(thr[2])
                    self.m4.append(thr[3])
                    updated_thr = True
            elif pkt_type == PKT_LINK and len(payload) >= LINK_STRUCT.size:
                link = LINK_STRUCT.unpack(payload[:LINK_STRUCT.size])
                self.last_rssi = link[0]
                self.last_loss = link[1]
                self.last_heartbeat = link[2]
                status_dirty = True

        now = time.monotonic()
        if now - self.last_fps_time >= 1.0:
            self.fps = self.imu_packets / (now - self.last_fps_time)
            self.imu_packets = 0
            self.last_fps_time = now
            status_dirty = True

        if updated_imu:
            self.accel_curves[0].setData(self.t_imu, self.ax)
            self.accel_curves[1].setData(self.t_imu, self.ay)
            self.accel_curves[2].setData(self.t_imu, self.az)
            self.gyro_curves[0].setData(self.t_imu, self.gx)
            self.gyro_curves[1].setData(self.t_imu, self.gy)
            self.gyro_curves[2].setData(self.t_imu, self.gz)
            self.angle_curves[0].setData(self.t_imu, self.roll)
            self.angle_curves[1].setData(self.t_imu, self.pitch)
            self.angle_curves[2].setData(self.t_imu, self.yaw)

        if updated_thr:
            self.throttle_curves[0].setData(self.t_thr, self.m1)
            self.throttle_curves[1].setData(self.t_thr, self.m2)
            self.throttle_curves[2].setData(self.t_thr, self.m3)
            self.throttle_curves[3].setData(self.t_thr, self.m4)

        if status_dirty:
            self._update_status_bar()

    def _update_status_bar(self) -> None:
        self.status.showMessage(
            f"FPS: {self.fps:.1f} | Packet loss: {self.last_loss} | RSSI: {self.last_rssi} dBm | Heartbeat: {self.last_heartbeat} ms"
        )

    def closeEvent(self, event) -> None:
        self.stop_event.set()
        self.serial_mgr.close()
        event.accept()


def main() -> None:
    parser = argparse.ArgumentParser(description="IMU realtime viewer")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate")
    args = parser.parse_args()

    serial_mgr = SerialManager(args.port, args.baud)
    stop_event = threading.Event()
    byte_queue = queue.Queue()
    packet_queue = queue.Queue()

    reader = SerialReaderThread(serial_mgr, byte_queue, stop_event)
    parser_thread = ParserThread(byte_queue, packet_queue, stop_event)
    heartbeat = HeartbeatThread(serial_mgr, stop_event)

    reader.start()
    parser_thread.start()
    heartbeat.start()

    app = QtWidgets.QApplication([])
    window = MainWindow(serial_mgr, packet_queue, stop_event)
    window.resize(1200, 700)
    window.show()
    app.exec_()

    stop_event.set()


if __name__ == "__main__":
    main()
