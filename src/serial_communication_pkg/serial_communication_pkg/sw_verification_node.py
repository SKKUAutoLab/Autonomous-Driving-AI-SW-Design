#!/usr/bin/env python3
import sys
import os

import cv2
if 'QT_QPA_PLATFORM_PLUGIN_PATH' in os.environ:
    os.environ.pop('QT_QPA_PLATFORM_PLUGIN_PATH')

import serial
import time
import math
import random
import serial.tools.list_ports
import threading
import subprocess
import signal
import yaml
import numpy as np

from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton,
                             QTextEdit, QVBoxLayout, QWidget, QMessageBox,
                             QHBoxLayout, QLabel, QStackedWidget, QCheckBox, QSlider, QLineEdit, QGridLayout, QComboBox, QGroupBox)
from PyQt5.QtCore import QTimer, pyqtSignal, Qt, pyqtSlot
from PyQt5.QtGui import QPixmap, QImage
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from interfaces_pkg.msg import MotionCommand, LaneInfo
from std_msgs.msg import String 

from .lib import protocol_convert_func_lib as PCFL

YOUR_ARDUINO_PORT = '/dev/ttyACM0' #아두이노 포트 번호 수정
BAUD_RATE = 115200 
JITTER_THRESHOLD_GOOD = 10.0   
JITTER_THRESHOLD_BAD = 20.0

class CameraPopupWindow(QWidget):
    """카메라 팝업 창"""
    window_closed = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Camera Preview")
        self.setGeometry(150, 150, 640, 480) 
        
        layout = QVBoxLayout()
        self.image_label = QLabel("Waiting for camera feed...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("background-color: #333; color: white;")
        layout.addWidget(self.image_label)
        self.setLayout(layout)

    @pyqtSlot(object)
    def update_image(self, qt_image):
        try:
            pixmap = QPixmap.fromImage(qt_image)
            self.image_label.setPixmap(pixmap.scaled(
                self.image_label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            ))
        except Exception as e:
            print(f"Camera Popup Error: {e}")

    def closeEvent(self, event):
        self.window_closed.emit()
        event.accept()

class LidarPopupWindow(QWidget):
    """LiDAR 팝업 창"""
    window_closed = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("LiDAR Visualization")
        self.setGeometry(150, 150, 700, 700)
        
        layout = QVBoxLayout()
        self.lidar_label = QLabel("Waiting for LiDAR data...")
        self.lidar_label.setAlignment(Qt.AlignCenter)
        self.lidar_label.setStyleSheet("background-color: #333; color: white;")
        layout.addWidget(self.lidar_label)
        self.setLayout(layout)

    @pyqtSlot(object)
    def update_image(self, qt_image):
        try:
            pixmap = QPixmap.fromImage(qt_image)
            self.lidar_label.setPixmap(pixmap.scaled(
                self.lidar_label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            ))
        except Exception as e:
            print(f"LiDAR Popup Error: {e}")

    def closeEvent(self, event):
        self.window_closed.emit()
        event.accept()

class CalibrationWidget(Node, QWidget): 
    """설정 및 캘리브레이션 페이지"""
    
    camera_frame_updated = pyqtSignal(object)
    lidar_image_updated = pyqtSignal(object)

    def __init__(self):
        Node.__init__(self, 'sw_verification_node') # 노드 이름만 변경
        QWidget.__init__(self)

        self.left_jitter = -1
        self.right_jitter = -1

        self.is_relaying_ros_commands = False

        self.bridge = CvBridge()

        self.camera_cap = None
        self.camera_timer = None
        self.lidar_timer = None
        self.latest_lidar_scan = None
        self.running_processes = []

        self.camera_popup = None
        self.lidar_popup = None
       
        self.serial_port = serial.Serial()
        self.serial_port.port = YOUR_ARDUINO_PORT 
        self.serial_port.baudrate = BAUD_RATE
        self.serial_port.timeout = 0.1 
     
        self.init_ui()
 
        self.serial_timer = QTimer(self)
        self.serial_timer.timeout.connect(self.read_serial_data)
        self.log_display.append("GUI가 시작되었습니다. 포트를 선택하고 연결하세요.")

        self.btn_refresh_ports.clicked.connect(self.refresh_ports)
        self.btn_connect.clicked.connect(self.connect_serial)
        self.btn_disconnect.clicked.connect(self.disconnect_serial)
      
        self.btn_auto_cal.clicked.connect(self.start_auto_calibration)
        self.btn_motor_test.clicked.connect(self.start_motor_test)

        self.start_button.clicked.connect(self.start_driving)
        self.stop_button.clicked.connect(self.stop_driving)

        self.stop_cal_button.clicked.connect(self.stop_calibration_or_test)
        
        self.steering_slider.valueChanged.connect(self.on_steering_slider_changed)

        self.btn_check_camera.clicked.connect(self.check_camera_connection)
        self.btn_show_camera.clicked.connect(self.toggle_camera_view)
        self.btn_check_lidar.clicked.connect(self.check_lidar_connection)
        self.btn_show_lidar.clicked.connect(self.toggle_lidar_view)

        self.refresh_ports()

        SUB_TOPIC_NAME = "topic_control_signal"
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, 
            history=QoSHistoryPolicy.KEEP_LAST, 
            durability=QoSDurabilityPolicy.VOLATILE, 
            depth=1
        )
        self.subscription = self.create_subscription(
            MotionCommand, 
            SUB_TOPIC_NAME, 
            self.ros_data_callback, 
            qos_profile
        )

        RAW_IMG_TOPIC_NAME = "image_raw"
        self.raw_image_sub = self.create_subscription(
            Image,
            RAW_IMG_TOPIC_NAME,
            self.raw_image_callback, 
            qos_profile
        )

        LIDAR_SCAN_TOPIC = "lidar_processed"
        self.lidar_scan_sub = self.create_subscription(
            LaserScan,
            LIDAR_SCAN_TOPIC,
            self.lidar_scan_callback,
            qos_profile
        )

    def lidar_scan_callback(self, msg):
        try:
            self.latest_lidar_scan = msg
        except Exception as e:
            self.get_logger().warn(f"LiDAR Scan callback error: {e}")

    def debug_image_callback(self, msg): 
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.debug_image_updated.emit(cv_image.copy()) 
        except Exception as e:
            self.get_logger().warn(f"Debug Image callback error: {e}")
       
    def raw_image_callback(self, msg):
        try:
            if msg.encoding == '8UC3':
                msg.encoding = 'bgr8'
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.raw_image_updated.emit(cv_image.copy()) 
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
        except Exception as e:
            self.get_logger().warn(f"Raw Image callback error: {e}")
       
    def ros_data_callback(self, msg):
        if not self.is_relaying_ros_commands:
            return       
        steering = msg.steering
        left_speed = msg.left_speed
        right_speed = msg.right_speed
        try:
            serial_msg = PCFL.convert_serial_message(steering, left_speed, right_speed)
            if self.serial_port.isOpen():
                self.serial_port.write(serial_msg.encode())
        except Exception as e:
            self.get_logger().error(f"ROS->Serial 변환/전송 오류: {e}")

    def init_ui(self):
        main_layout = QGridLayout() 

        # 1. 아두이노
        group_connection = QGroupBox("1. 아두이노")
        conn_layout = QGridLayout()
        group_connection.setLayout(conn_layout)
        port_label = QLabel("Port:")
        self.port_combo = QComboBox()
        self.btn_refresh_ports = QPushButton("Refresh")
        conn_layout.addWidget(port_label, 0, 0)
        conn_layout.addWidget(self.port_combo, 0, 1)
        conn_layout.addWidget(self.btn_refresh_ports, 0, 2)
        self.btn_connect = QPushButton("Connect Arduino")
        self.btn_connect.setStyleSheet("background-color: lightgreen;")
        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_disconnect.setStyleSheet("background-color: salmon;")
        self.btn_disconnect.setEnabled(False)
        conn_layout.addWidget(self.btn_connect, 1, 0, 1, 2)
        conn_layout.addWidget(self.btn_disconnect, 1, 2)
        main_layout.addWidget(group_connection, 0, 0, 1, 4) 

        # 2. 가변저항
        group_steering = QGroupBox("2. 가변저항")
        steering_layout = QGridLayout()

        cal_button_layout = QHBoxLayout()
        self.btn_auto_cal = QPushButton("Auto Cal (a)")
        cal_button_layout.addWidget(self.btn_auto_cal)
        
        steering_layout.addLayout(cal_button_layout, 0, 0, 1, 4)
        
        self.start_button = QPushButton("Start Driving (d)")
        self.stop_button = QPushButton("Stop Driving (p)")
        self.stop_button.setEnabled(False) 
        steering_layout.addWidget(self.start_button, 1, 0, 1, 2)
        steering_layout.addWidget(self.stop_button, 1, 2, 1, 2)

        self.steering_slider = QSlider(Qt.Horizontal) 
        self.steering_slider.setRange(-7, 7)
        self.steering_slider.setValue(0)
        self.steering_slider.setEnabled(False) 
        self.steering_slider.setTickPosition(QSlider.TicksBelow)
        self.steering_slider.setTickInterval(1)
        steering_layout.addWidget(self.steering_slider, 2, 0, 1, 4)

        # 3. 모터
        group_steering.setLayout(steering_layout)
        main_layout.addWidget(group_steering, 1, 0, 1, 4)
        group_motor = QGroupBox("3. 모터")
        motor_layout = QVBoxLayout()
        self.btn_motor_test = QPushButton("Motor Test (m)")
        motor_layout.addWidget(self.btn_motor_test)
        self.stop_cal_button = QPushButton("EMERGENCY STOP (e)")
        self.stop_cal_button.setEnabled(False) 
        motor_layout.addWidget(self.stop_cal_button)
        group_motor.setLayout(motor_layout)
        main_layout.addWidget(group_motor, 2, 0, 1, 2)

        # 4. 카메라
        group_camera = QGroupBox("4. 카메라")
        camera_layout = QGridLayout()
        self.btn_check_camera = QPushButton("Camera Connection Check")
        self.btn_check_camera.setStyleSheet("background-color: lightblue;")
        camera_layout.addWidget(self.btn_check_camera, 0, 0, 1, 2)

        camera_control_layout = QHBoxLayout()
        self.camera_num_input = QLineEdit("0")
        self.camera_num_input.setMaximumWidth(50)
        camera_control_layout.addWidget(QLabel("Camera #:"))
        camera_control_layout.addWidget(self.camera_num_input)
        self.btn_show_camera = QPushButton("Show Camera")
        self.btn_show_camera.setStyleSheet("background-color: lightgreen;")
        camera_control_layout.addWidget(self.btn_show_camera)
        camera_layout.addLayout(camera_control_layout, 1, 0, 1, 2)
        group_camera.setLayout(camera_layout)
        main_layout.addWidget(group_camera, 2, 2, 1, 2)

        # 5. 라이다
        group_lidar = QGroupBox("5. 라이다")
        lidar_layout = QGridLayout()
        self.btn_check_lidar = QPushButton("LiDAR Connection Check")
        self.btn_check_lidar.setStyleSheet("background-color: lightblue;")
        lidar_layout.addWidget(self.btn_check_lidar, 0, 0, 1, 2)

        lidar_control_layout = QHBoxLayout()
        self.lidar_num_input = QLineEdit("0")
        self.lidar_num_input.setMaximumWidth(50)
        lidar_control_layout.addWidget(QLabel("LiDAR #:"))
        lidar_control_layout.addWidget(self.lidar_num_input)
        self.btn_show_lidar = QPushButton("Show LiDAR Data")
        self.btn_show_lidar.setStyleSheet("background-color: lightgreen;")
        lidar_control_layout.addWidget(self.btn_show_lidar)
        lidar_layout.addLayout(lidar_control_layout, 1, 0, 1, 2)

        angle_range_layout = QHBoxLayout()
        angle_range_layout.addWidget(QLabel("각도 범위:"))
        self.lidar_angle_start = QLineEdit("0")
        self.lidar_angle_start.setMaximumWidth(50)
        angle_range_layout.addWidget(self.lidar_angle_start)
        angle_range_layout.addWidget(QLabel("~"))
        self.lidar_angle_end = QLineEdit("360")
        self.lidar_angle_end.setMaximumWidth(50)
        angle_range_layout.addWidget(self.lidar_angle_end)
        angle_range_layout.addWidget(QLabel("°"))
        lidar_layout.addLayout(angle_range_layout, 2, 0, 1, 2)
        group_lidar.setLayout(lidar_layout)
        main_layout.addWidget(group_lidar, 3, 0, 1, 4) 

        # 7. 상태 라벨 및 로그
        self.status_label = QLabel("상태: 대기 중")
        self.status_label.setStyleSheet("font-weight: bold;")
        main_layout.addWidget(self.status_label, 4, 0, 1, 4) 
        
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        main_layout.addWidget(self.log_display, 5, 0, 1, 4) 

        main_layout.setRowStretch(5, 1) 
        self.setLayout(main_layout)

    def refresh_ports(self):
        self.log_display.append("시리얼 포트 목록 새로고침...")
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        found_ports = []
        for port in ports:
            if 'ttyACM' in port.device:
                self.port_combo.addItem(port.device)
                found_ports.append(port.device)
        if found_ports:
            self.log_display.append(f"사용 가능한 포트: {', '.join(found_ports)}")
            self.btn_connect.setEnabled(True)
        else:
            self.log_display.append("아두이노 연결을 확인하세요")
            self.btn_connect.setEnabled(False)

    def connect_serial(self):
        if self.serial_port.isOpen():
            self.log_display.append("이미 연결되어 있습니다.")
            return
        selected_port = self.port_combo.currentText()
        if not selected_port:
            self.log_display.append("❌ 선택된 포트가 없습니다. 'Refresh'를 누르세요.")
            return
        try:
            self.serial_port.port = selected_port
            self.serial_port.open()
            self.log_display.append(f"{selected_port} 포트 여는 중... 장치 확인 중...")
            time.sleep(1.0) 
            self.serial_port.write(b'?\n')
            response = self.serial_port.readline().decode('utf-8').strip()
            
            if response == "ARDUINO_READY": 
                self.serial_timer.start(100) 
                self.log_display.append("✅ 아두이노 확인 완료. 연결 성공.")
                self.set_driving_mode()
                self.status_label.setText("상태: 연결됨")
                self.status_label.setStyleSheet("font-weight: bold; color: green;")
                self.btn_connect.setEnabled(False)
                self.btn_disconnect.setEnabled(True)
            else:
                self.serial_port.close()
                self.log_display.append(f"❌ 연결 실패: 아두이노 응답 없음 (응답: {response})")
                self.status_label.setText("상태: 장치 확인 실패")
                self.status_label.setStyleSheet("font-weight: bold; color: red;")
        except serial.SerialException as e:
            self.log_display.append(f"❌ {selected_port} 포트 열기 실패!")
            self.log_display.append(f"  {e}")
            self.status_label.setText("상태: 연결 실패")
            self.status_label.setStyleSheet("font-weight: bold; color: red;")
        except Exception as e:
             self.log_display.append(f"❌ 알 수 없는 연결 오류: {e}")
             if self.serial_port.isOpen():
                 self.serial_port.close()

    def disconnect_serial(self):
        if self.serial_port.isOpen():
            try:
                self.send_command('p') 
                time.sleep(0.1)
                self.serial_timer.stop()
                self.serial_port.close()
                self.log_display.append("시리얼 포트 연결 해제됨.")
            except Exception as e:
                self.log_display.append(f"해제 중 오류: {e}")
        else:
            self.log_display.append("이미 연결이 해제되어 있습니다.")
        self.status_label.setText("상태: 연결 대기 중")
        self.status_label.setStyleSheet("font-weight: bold; color: black;")
        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)
        self.refresh_ports()

    def public_close_port(self):
        self.get_logger().info("앱 종료... 모터 정지 및 포트 닫기.")
        self.disconnect_serial()
        if self.camera_popup:
            self.camera_popup.close()
        if self.lidar_popup:
            self.lidar_popup.close()

    def send_command(self, command_str):
        if self.serial_port.isOpen():
            try:
                command_bytes = (command_str + '\n').encode('utf-8')
                self.serial_port.write(command_bytes)
                
            except serial.SerialException as e:
                 self.log_display.append(f"❌ 시리얼 쓰기 오류: {e}")
                 self.disconnect_serial()
        else:
            self.log_display.append("오류: 시리얼 포트가 열려있지 않습니다.")
            self.status_label.setText("상태: 🔴 연결 필요")
            self.status_label.setStyleSheet("font-weight: bold; color: red;")

    def start_calibration_common(self, command_char, status_text):
        self.is_relaying_ros_commands = False 
        self.left_jitter = -1
        self.right_jitter = -1
        self.status_label.setText(f"상태: {status_text} 진행 중...")
        self.status_label.setStyleSheet("font-weight: bold; color: blue;")
        self.send_command(command_char)
        self.set_buttons_enabled(False)
        self.stop_cal_button.setEnabled(True) 
        self.log_display.append(f"... 아두이노에서 {status_text} 진행 중 ...")

    def start_auto_calibration(self):
        command_str = "a" 
        self.start_calibration_common(command_str, "자동 캘리브레이션")
   
    def start_motor_test(self):
        self.status_label.setText("상태: 모터 테스트 진행 중...")
        self.status_label.setStyleSheet("font-weight: bold; color: blue;")
        self.send_command('m')
        self.set_buttons_enabled(False) 
        self.stop_cal_button.setEnabled(True)

    def stop_calibration_or_test(self):
        self.send_command('e') 
        self.is_relaying_ros_commands = False 
        self.status_label.setText("상태: 작업 강제 중지됨")
        self.status_label.setStyleSheet("font-weight: bold; color: salmon;")
        self.set_buttons_enabled(True)

    def on_steering_slider_changed(self, value):
        if not self.steering_slider.isEnabled():
            return
        command_str = f"s{value}l0r0"
        self.send_command(command_str) 
        self.status_label.setText(f"수동 제어: Angle = {value}")

    def start_driving(self): 
        self.send_command('d')  
        self.is_relaying_ros_commands = False 
        self.steering_slider.setEnabled(True)   
        self.steering_slider.setValue(0)
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.set_buttons_enabled(False)
        self.status_label.setText("상태: 수동 제어 모드 (Manual Mode)")
        
    def stop_driving(self): 
        self.send_command('p')
        self.is_relaying_ros_commands = False
        self.steering_slider.setEnabled(False)   
        self.steering_slider.setValue(0)        
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.set_buttons_enabled(True)
        self.status_label.setText("상태: 수동 제어 모드 OFF")

    def set_buttons_enabled(self, enabled):
        self.btn_auto_cal.setEnabled(enabled)
        self.btn_motor_test.setEnabled(enabled)
        self.stop_cal_button.setEnabled(False)

    def set_driving_mode(self):
        self.steering_slider.setRange(-7, 7)
        self.steering_slider.setTickInterval(1)
        self.send_command("M7")
        self.steering_slider.setValue(0)

    def display_final_calibration_status(self):
        if self.left_jitter == -1 or self.right_jitter == -1:
            self.status_label.setText("캘리브레이션 완료 (Jitter 값 없음)")
            self.status_label.setStyleSheet("font-weight: bold; color: gray;")
            return

        final_text = f"최종 Jitter: [좌: {self.left_jitter}, 우: {self.right_jitter}]"
        if self.left_jitter >= JITTER_THRESHOLD_BAD or self.right_jitter >= JITTER_THRESHOLD_BAD:
            self.status_label.setText(f"{final_text} - 🟡 경고 (센서 노이즈 감지)")
            self.status_label.setStyleSheet("font-weight: bold; color: orange;")
        elif self.left_jitter <= JITTER_THRESHOLD_GOOD and self.right_jitter <= JITTER_THRESHOLD_GOOD:
            self.status_label.setText(f"{final_text} - 🟢 양호")
            self.status_label.setStyleSheet("font-weight: bold; color: green;")
        else:
            self.status_label.setText(f"{final_text} - 🟡 경고 (센서 노이즈 감지)")
            self.status_label.setStyleSheet("font-weight: bold; color: orange;")

    def read_serial_data(self):
       if self.serial_port.isOpen():
            try:
                line = self.serial_port.readline()
                if not line: return 
                text = line.decode('utf-8', errors='ignore').strip()
                if text:
                    self.log_display.append(f"Arduino: {text}")
                    if "Left value saved:" in text:
                        try:
                            jitter_part = text.split(',')[-1]
                            jitter_val = int(jitter_part.split(':')[-1].strip())
                            self.left_jitter = jitter_val
                        except Exception as e:
                            self.log_display.append(f"Jitter 파싱 오류: {e}")
                    elif "Right value saved:" in text:
                        try:
                            jitter_part = text.split(',')[-1]
                            jitter_val = int(jitter_part.split(':')[-1].strip())
                            self.right_jitter = jitter_val
                        except Exception as e:
                            self.log_display.append(f"Jitter 파싱 오류: {e}")
                    elif "Centering complete!" in text:
                         self.log_display.append("--- 캘리브레이션 완료 ---")
                         self.set_buttons_enabled(True) 
                         self.display_final_calibration_status()
                    elif "Calibration FAILED!" in text:
                         self.log_display.append("--- 캘리브레이션 실패 ---")
                         self.set_buttons_enabled(True)
                         if "GND" in text or "VCC" in text or "Analog" in text: 
                             self.status_label.setText("🔴 FAILED: 핀 연결 불량")
                         elif "timeout" in text.lower(): 
                             self.status_label.setText("🔴 FAILED: 타임아웃")
                         else: 
                             self.status_label.setText("🔴 캘리브레이션 실패 (로그 확인)")
                         self.status_label.setStyleSheet("font-weight: bold; color: red;")
                    elif "ABORTED!" in text: 
                         if "Motor Test" in text:
                             self.log_display.append("--- 모터 테스트 중지됨 ---")
                         else:
                             self.log_display.append("--- 캘리브레이션 중지됨 ---")
                         self.set_buttons_enabled(True)
                         self.status_label.setText("상태: 작업 강제 중지됨")
                         self.status_label.setStyleSheet("font-weight: bold; color: red;")
                    elif "Motor Test Complete!" in text:
                         self.log_display.append("--- 모터 테스트 완료 ---")
                         self.set_buttons_enabled(True)
                         self.status_label.setText("상태: 모터 테스트 완료")
                         self.status_label.setStyleSheet("font-weight: bold; color: green;")
                    elif "Driving Mode ON" in text:
                        self.status_label.setText("상태: 주행 중 (Driving ON)")
                        self.status_label.setStyleSheet("font-weight: bold; color: green;")
                    elif "Driving Mode OFF" in text:
                        self.status_label.setText("상태: 주행 중지 (Driving OFF)")
                        self.status_label.setStyleSheet("font-weight: bold; color: gray;")
            except Exception as e:
                print(f"시리얼 읽기 오류: {e}")
                self.disconnect_serial()

    def start_process(self, command_list):
        try:
            process = subprocess.Popen(command_list, preexec_fn=os.setsid)
            self.running_processes.append(process)
            self.log_display.append(f"✅ [PID: {process.pid}] 실행: {' '.join(command_list)}")
        except Exception as e:
            self.log_display.append(f"❌ 실행 오류: {e}")

    def check_camera_connection(self):
        self.log_display.append("=== Camera Connection Check ===")
        try:
            result = subprocess.run("ls -l /dev/video*", capture_output=True, text=True, shell=True)
            if result.returncode == 0:
                self.log_display.append(result.stdout)
                if '/dev/video' in result.stdout:
                    self.log_display.append(f"✓ 발견된 카메라 장치 있음")
                else:
                    self.log_display.append("✗ 카메라 장치를 찾을 수 없습니다.")
            else:
                self.log_display.append("✗ 카메라 장치를 찾을 수 없습니다.")
        except Exception as e:
            self.log_display.append(f"✗ 오류 발생: {str(e)}")
        self.log_display.append("")

    def check_lidar_connection(self):
        self.log_display.append("=== LiDAR Connection Check ===")
        try:
            result = subprocess.run("ls -l /dev/ttyUSB*", capture_output=True, text=True, shell=True)
            if result.returncode == 0:
                self.log_display.append(result.stdout)
                if '/dev/ttyUSB' in result.stdout:
                    self.log_display.append(f"✓ 발견된 USB 장치 있음")
                else:
                    self.log_display.append("✗ USB 장치를 찾을 수 없습니다.")
            else:
                self.log_display.append("✗ USB 장치를 찾을 수 없습니다.")
        except Exception as e:
            self.log_display.append(f"✗ 오류 발생: {str(e)}")
        self.log_display.append("")

    def toggle_camera_view(self):
        if self.camera_popup is None:
            try:
                cam_num = int(self.camera_num_input.text())
                self.camera_cap = cv2.VideoCapture(cam_num)
                self.camera_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.camera_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                if not self.camera_cap.isOpened():
                    self.log_display.append(f"✗ 카메라 {cam_num}을 열 수 없습니다.")
                    self.camera_cap = None
                    return
                self.log_display.append(f"✓ 카메라 {cam_num} 시작")
                self.camera_popup = CameraPopupWindow()
                self.camera_frame_updated.connect(self.camera_popup.update_image)
                self.camera_popup.window_closed.connect(self.stop_camera)
                self.camera_popup.show()
                self.camera_timer = QTimer()
                self.camera_timer.timeout.connect(self.update_camera_frame)
                self.camera_timer.start(30)
                self.btn_show_camera.setText("Stop Camera")
                self.btn_show_camera.setStyleSheet("background-color: salmon;")
            except Exception as e:
                self.log_display.append(f"✗ 카메라 시작 실패: {str(e)}")
                if self.camera_cap: self.camera_cap.release()
                self.camera_cap = None
                self.camera_popup = None
        else:
            self.camera_popup.close()

    @pyqtSlot()
    def update_camera_frame(self):
        if self.camera_cap is not None:
            ret, frame = self.camera_cap.read()
            if ret:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = frame_rgb.shape
                bytes_per_line = ch * w
                qt_image = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
                self.camera_frame_updated.emit(qt_image)

    @pyqtSlot()
    def stop_camera(self):
        if self.camera_timer is not None:
            self.camera_timer.stop()
            self.camera_timer = None
        if self.camera_cap is not None:
            self.camera_cap.release()
            self.camera_cap = None
        if self.camera_popup:
            try:
                self.camera_frame_updated.disconnect(self.camera_popup.update_image)
                self.camera_popup.window_closed.disconnect(self.stop_camera)
            except TypeError: pass
            self.camera_popup = None
        self.btn_show_camera.setText("Show Camera")
        self.btn_show_camera.setStyleSheet("background-color: lightgreen;")
        self.log_display.append("카메라 중지")

    def toggle_lidar_view(self):
        if self.lidar_popup is None:
            try:
                self.log_display.append("=== LiDAR 노드 시작 ===")
                cmd_publisher = ['ros2', 'run', 'lidar_perception_pkg', 'lidar_publisher_node']
                self.start_process(cmd_publisher)
                cmd_processor = ['ros2', 'run', 'lidar_perception_pkg', 'lidar_processor_node']
                self.start_process(cmd_processor)
                self.lidar_popup = LidarPopupWindow()
                self.lidar_image_updated.connect(self.lidar_popup.update_image)
                self.lidar_popup.window_closed.connect(self.stop_lidar)
                self.lidar_popup.show()
                self.lidar_timer = QTimer()
                self.lidar_timer.timeout.connect(self.update_lidar_visualization)
                self.lidar_timer.start(100)
                self.btn_show_lidar.setText("Stop LiDAR")
                self.btn_show_lidar.setStyleSheet("background-color: salmon;")
                self.log_display.append("✓ LiDAR 데이터 시각화 시작...")
            except Exception as e:
                self.log_display.append(f"✗ LiDAR 시각화 시작 실패: {str(e)}")
                self.lidar_timer = None
                self.lidar_popup = None
        else:
            self.lidar_popup.close()

    @pyqtSlot()
    def update_lidar_visualization(self):
        if self.latest_lidar_scan is not None:
            qt_image = self.visualize_lidar(self.latest_lidar_scan)
            if qt_image:
                self.lidar_image_updated.emit(qt_image)
        else:
            qt_image = self.visualize_lidar_dummy()
            if qt_image:
                self.lidar_image_updated.emit(qt_image)

    def visualize_lidar(self, scan_msg):
        try:
            size = 700
            img = np.zeros((size, size, 3), dtype=np.uint8)
            img[:] = (20, 20, 20)
            center = (size // 2, size // 2)
            cv2.circle(img, center, 8, (0, 255, 0), -1)
            ranges = scan_msg.ranges
            angle_min = scan_msg.angle_min
            angle_increment = scan_msg.angle_increment
            max_display_distance = 1.0
            scale = 300.0 / max_display_distance
            try:
                angle_start_deg = float(self.lidar_angle_start.text())
                angle_end_deg = float(self.lidar_angle_end.text())
            except:
                angle_start_deg = 0
                angle_end_deg = 360
            for i, distance in enumerate(ranges):
                if not math.isfinite(distance) or distance == 0: continue
                angle_rad = angle_min + i * angle_increment
                if distance > max_display_distance: continue
                display_angle = -(angle_rad - math.pi / 2)
                angle_deg = math.degrees(display_angle) % 360
                if angle_start_deg <= angle_end_deg:
                    if not (angle_start_deg <= angle_deg <= angle_end_deg): continue
                else:
                    if not (angle_deg >= angle_start_deg or angle_deg <= angle_end_deg): continue
                x = int(center[0] + distance * scale * math.cos(display_angle))
                y = int(center[1] + distance * scale * math.sin(display_angle))
                if 0 <= x < size and 0 <= y < size:
                    if distance < 0.3: color = (0, 0, 255)
                    elif distance < 0.6: color = (0, 255, 255)
                    else: color = (255, 255, 0)
                    cv2.circle(img, (x, y), 2, color, -1) 
            for dist_cm in range(10, 110, 10):
                dist = dist_cm / 100.0
                radius = int(dist * scale)
                cv2.circle(img, center, radius, (60, 60, 60), 1)
            return self._cv2_to_qimage(img)
        except Exception as e:
            self.log_display.append(f"LiDAR viz error: {e}")
            return None

    def visualize_lidar_dummy(self):
        try:
            size = 500
            img = np.zeros((size, size, 3), dtype=np.uint8)
            img[:] = (40, 40, 40)
            center = (size // 2, size // 2)
            cv2.circle(img, center, 5, (0, 255, 0), -1)
            for angle in range(0, 360, 2):
                distance = random.uniform(50, 200)
                rad = np.deg2rad(angle)
                x = int(center[0] + distance * np.cos(rad))
                y = int(center[1] + distance * np.sin(rad))
                cv2.circle(img, (x, y), 2, (0, 255, 255), -1)
            return self._cv2_to_qimage(img)
        except Exception as e:
            self.log_display.append(f"LiDAR dummy viz error: {e}")
            return None

    def _cv2_to_qimage(self, cv_img):
        img_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = img_rgb.shape
        bytes_per_line = ch * w
        return QImage(img_rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)

    @pyqtSlot()
    def stop_lidar(self):
        if self.lidar_timer is not None:
            self.lidar_timer.stop()
            self.lidar_timer = None
        if self.running_processes:
            self.log_display.append("=== LiDAR 노드 종료 중 ===")
            for proc in self.running_processes:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except Exception: pass
            self.running_processes.clear()
        if self.lidar_popup:
            try:
                self.lidar_image_updated.disconnect(self.lidar_popup.update_image)
                self.lidar_popup.window_closed.disconnect(self.stop_lidar)
            except TypeError: pass
            self.lidar_popup = None
        self.btn_show_lidar.setText("Show LiDAR Data")
        self.btn_show_lidar.setStyleSheet("background-color: lightgreen;")
        self.log_display.append("LiDAR 중지 완료")
        self.latest_lidar_scan = None

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SW Verification Node")
        self.resize(800, 700)
       
        self.calibration_widget = CalibrationWidget()
        self.setCentralWidget(self.calibration_widget)
  
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.calibration_widget,))
        self.ros_thread.daemon = True
        self.ros_thread.start()

    def closeEvent(self, event):
        self.calibration_widget.public_close_port()
        rclpy.shutdown()
        self.ros_thread.join(timeout=1.0)
        event.accept()

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
   
    signal.signal(signal.SIGINT, lambda *args: app.quit())
    
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()