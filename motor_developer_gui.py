"""
모터 개발자 도구 - PyQt GUI 버전 (UART 통신)
FIT0405 모터 + SJ01 엔코더 + L298N 드라이버용
ESP32와 UART 통신으로 데이터 주고받기
"""

import sys
import time
import math
import threading
import serial
import json
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QGridLayout, QLabel, QPushButton, 
                             QLineEdit, QTextEdit, QTabWidget, QGroupBox,
                             QSlider, QSpinBox, QDoubleSpinBox, QProgressBar,
                             QTableWidget, QTableWidgetItem, QComboBox,
                             QCheckBox, QMessageBox, QFileDialog)
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, Qt
from PyQt5.QtGui import QFont, QPalette, QColor
import pyqtgraph as pg

class UARTMotorController:
    """UART를 통한 모터 제어 클래스"""
    
    def __init__(self, port='COM3', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.connected = False
        
    def connect(self):
        """UART 연결"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            time.sleep(2)  # 연결 안정화 대기
            self.connected = True
            return True
        except Exception as e:
            print(f"UART 연결 실패: {e}")
            self.connected = False
            return False
            
    def disconnect(self):
        """UART 연결 해제"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.connected = False
        
    def send_command(self, command, data=None):
        """ESP32에 명령 전송"""
        if not self.connected:
            print(f"UART 연결되지 않음 - 명령: {command}")
            return None
            
        try:
            message = {
                'command': command,
                'data': data,
                'timestamp': time.time()
            }
            
            json_str = json.dumps(message) + '\n'
            print(f"전송할 JSON: {json_str.strip()}")  # 디버깅 활성화
            
            # 데이터 전송
            bytes_sent = self.serial_conn.write(json_str.encode())
            print(f"전송된 바이트 수: {bytes_sent}")
            
            # 전송 후 잠시 대기
            time.sleep(0.1)
            
            # 응답 대기 (타임아웃 증가)
            self.serial_conn.timeout = 2  # 2초로 증가
            response = self.serial_conn.readline()
            
            if response:
                response_str = response.decode().strip()
                print(f"수신된 응답: {response_str}")
                return json.loads(response_str)
            else:
                print("응답 없음 - ESP32에서 데이터를 수신하지 못함")
                return None
            
        except Exception as e:
            print(f"명령 전송 실패: {e}")
            return None
            
    def forward(self, speed):
        """정방향 회전"""
        return self.send_command('forward', {'speed': speed})
        
    def backward(self, speed):
        """역방향 회전"""
        return self.send_command('backward', {'speed': speed})
        
    def stop(self):
        """모터 정지"""
        return self.send_command('stop')
        
    def reset_position(self):
        """위치 리셋"""
        return self.send_command('reset_position')
        
    def get_position(self):
        """현재 위치 가져오기"""
        response = self.send_command('get_position')
        if response and 'data' in response:
            return response['data'].get('position', 0)
        return 0
        
    def get_current_rpm(self):
        """현재 RPM 가져오기"""
        response = self.send_command('get_rpm')
        if response and 'data' in response:
            return response['data'].get('rpm', 0.0)
        return 0.0
        
    def get_direction(self):
        """현재 방향 가져오기"""
        response = self.send_command('get_direction')
        if response and 'data' in response:
            return response['data'].get('direction', False)
        return False
        
    def set_speed_pid_parameters(self, kp, ki, kd):
        """속도 PID 파라미터 설정"""
        return self.send_command('set_speed_pid', {'kp': kp, 'ki': ki, 'kd': kd})
        
    def set_position_pid_parameters(self, kp, ki, kd):
        """위치 PID 파라미터 설정"""
        return self.send_command('set_position_pid', {'kp': kp, 'ki': ki, 'kd': kd})
        
    def control_speed_pid(self, target_rpm):
        """PID 속도 제어"""
        response = self.send_command('control_speed_pid', {'target_rpm': target_rpm})
        if response and 'data' in response:
            return response['data'].get('current_rpm', 0.0)
        return 0.0
        
    def control_position_pid(self, target_position):
        """PID 위치 제어"""
        response = self.send_command('control_position_pid', {'target_position': target_position})
        if response and 'data' in response:
            return response['data'].get('current_position', 0)
        return 0

class DataLogger(QThread):
    """데이터 로깅 스레드"""
    data_updated = pyqtSignal(float, float, float, float, float, float, float)
    
    def __init__(self, motor):
        super().__init__()
        self.motor = motor
        self.is_logging = False
        self.target_rpm = 0
        self.kp = 0
        self.ki = 0
        self.kd = 0
        
    def start_logging(self, target_rpm, kp, ki, kd):
        self.target_rpm = target_rpm
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.is_logging = True
        self.start()
        
    def stop_logging(self):
        self.is_logging = False
        self.quit()
        self.wait()
        
    def run(self):
        start_time = time.time()
        while self.is_logging:
            current_time = time.time() - start_time
            current_rpm = self.motor.get_current_rpm()
            current_pos = self.motor.get_position()
            
            self.data_updated.emit(
                current_time, self.target_rpm, current_rpm, 
                self.kp, self.ki, self.kd, current_pos
            )
            
            time.sleep(2.0)  # 10년 전 컴퓨터 최적화: 2초로 대폭 증가

class MotorDeveloperGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        # ESP32 USB 포트: COM3, 디버깅 터미널 포트: COM3
        self.motor = UARTMotorController('COM3', 115200)
        self.data_logger = DataLogger(self.motor)
        self.data_logger.data_updated.connect(self.update_data)
        
        # 메모리 최적화를 위한 데이터 구조
        self.data_history = []
        self.is_testing = False
        self.is_auto_tuning = False  # 자동 튜닝 상태 관리
        
        # 성능 최적화를 위한 캐시 변수들
        self._last_rpm = 0
        self._last_pos = 0
        self._last_log_time = 0
        self._last_graph_update = 0
        
        self.init_ui()
        self.setup_connections()
        
        # UART 연결 시도 제거 - 처음에는 연결하지 않은 상태로 시작
        # self.connect_uart()
        
    def connect_uart(self):
        """UART 연결"""
        # 기존 연결 해제
        self.motor.disconnect()
        
        # 새로운 설정으로 연결
        port = self.port_combo.currentText()
        baudrate = int(self.baudrate_combo.currentText())
        
        self.motor = UARTMotorController(port, baudrate)
        self.data_logger = DataLogger(self.motor)
        self.data_logger.data_updated.connect(self.update_data)
        
        if self.motor.connect():
            self.log_message(f"UART 연결 성공 - {port} @ {baudrate} (ESP32 USB 포트)")
            self.log_message("ESP32 UART0 핀: TX=GPIO1, RX=GPIO3")
            self.statusBar().showMessage(f"UART 연결됨 - {port} (ESP32 USB 포트)")
            
            # 연결 테스트
            self.log_message("ESP32 연결 테스트 중...")
            test_response = self.motor.send_command('ping')
            if test_response:
                self.log_message("ESP32 연결 테스트 성공!")
            else:
                self.log_message("ESP32 연결 테스트 실패 - 명령 전송 불가")
        else:
            self.log_message(f"UART 연결 실패 - {port} @ {baudrate} (ESP32 USB 포트)")
            self.log_message("ESP32 UART0 핀 확인 필요: TX=GPIO1, RX=GPIO3")
            self.statusBar().showMessage("UART 연결 실패")
            
    def disconnect_uart(self):
        """UART 연결 해제"""
        self.motor.disconnect()
        self.log_message("UART 연결 해제")
        self.statusBar().showMessage("UART 연결 해제됨")
        
    def init_ui(self):
        """UI 초기화"""
        self.setWindowTitle("모터 개발자 도구 - GUI")
        self.setGeometry(100, 100, 1200, 800)
        
        # 중앙 위젯
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 메인 레이아웃
        main_layout = QHBoxLayout(central_widget)
        
        # 왼쪽 패널 (컨트롤)
        left_panel = self.create_control_panel()
        main_layout.addWidget(left_panel, 1)
        
        # 오른쪽 패널 (그래프)
        right_panel = self.create_graph_panel()
        main_layout.addWidget(right_panel, 2)
        
        # 상태바
        self.statusBar().showMessage("준비")
        
    def create_control_panel(self):
        """컨트롤 패널 생성"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # 탭 위젯
        tab_widget = QTabWidget()
        
        # 기본 제어 탭
        basic_tab = self.create_basic_control_tab()
        tab_widget.addTab(basic_tab, "기본 제어")
        
        # PID 튜닝 탭
        pid_tab = self.create_pid_tuning_tab()
        tab_widget.addTab(pid_tab, "PID 튜닝")
        
        # 위치 제어 탭
        position_tab = self.create_position_control_tab()
        tab_widget.addTab(position_tab, "위치 제어")
        
        # 분석 탭
        analysis_tab = self.create_analysis_tab()
        tab_widget.addTab(analysis_tab, "분석")
        
        layout.addWidget(tab_widget)
        
        # 로그 출력
        log_group = QGroupBox("로그")
        log_layout = QVBoxLayout(log_group)
        
        self.log_text = QTextEdit()
        self.log_text.setMaximumHeight(150)
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        
        layout.addWidget(log_group)
        
        return panel
        
    def create_basic_control_tab(self):
        """기본 제어 탭 생성"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # UART 연결 그룹
        uart_group = QGroupBox("UART 연결 (ESP32 UART0: GPIO1(TX), GPIO3(RX))")
        uart_layout = QGridLayout(uart_group)
        
        uart_layout.addWidget(QLabel("포트:"), 0, 0)
        self.port_combo = QComboBox()
        self.port_combo.addItems(['COM1', 'COM2', 'COM3', 'COM4', 'COM5', 'COM6', 'COM7', 'COM8'])
        self.port_combo.setCurrentText('COM3')
        uart_layout.addWidget(self.port_combo, 0, 1)
        
        uart_layout.addWidget(QLabel("보드레이트:"), 1, 0)
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(['9600', '19200', '38400', '57600', '115200'])
        self.baudrate_combo.setCurrentText('115200')
        uart_layout.addWidget(self.baudrate_combo, 1, 1)
        
        self.connect_btn = QPushButton("연결")
        self.connect_btn.clicked.connect(self.connect_uart)
        uart_layout.addWidget(self.connect_btn, 2, 0)
        
        self.disconnect_btn = QPushButton("연결 해제")
        self.disconnect_btn.clicked.connect(self.disconnect_uart)
        uart_layout.addWidget(self.disconnect_btn, 2, 1)
        
        layout.addWidget(uart_group)
        
        # 모터 제어 그룹
        motor_group = QGroupBox("모터 제어")
        motor_layout = QGridLayout(motor_group)
        
        # 속도 설정
        motor_layout.addWidget(QLabel("속도 (%):"), 0, 0)
        self.speed_spinbox = QSpinBox()
        self.speed_spinbox.setRange(0, 100)
        self.speed_spinbox.setValue(50)
        motor_layout.addWidget(self.speed_spinbox, 0, 1)
        
        # 제어 버튼들
        self.forward_btn = QPushButton("정방향")
        self.forward_btn.clicked.connect(self.forward_motor)
        motor_layout.addWidget(self.forward_btn, 1, 0)
        
        self.backward_btn = QPushButton("역방향")
        self.backward_btn.clicked.connect(self.backward_motor)
        motor_layout.addWidget(self.backward_btn, 1, 1)
        
        self.stop_btn = QPushButton("정지")
        self.stop_btn.clicked.connect(self.stop_motor)
        motor_layout.addWidget(self.stop_btn, 1, 2)
        
        layout.addWidget(motor_group)
        
        # 엔코더 테스트 그룹
        encoder_group = QGroupBox("엔코더 테스트")
        encoder_layout = QVBoxLayout(encoder_group)
        
        self.encoder_test_btn = QPushButton("엔코더 펄스 테스트")
        self.encoder_test_btn.clicked.connect(self.test_encoder)
        encoder_layout.addWidget(self.encoder_test_btn)
        
        self.reset_position_btn = QPushButton("위치 리셋")
        self.reset_position_btn.clicked.connect(self.reset_position)
        encoder_layout.addWidget(self.reset_position_btn)
        
        # 하드웨어 테스트 버튼 추가
        self.test_motor_btn = QPushButton("모터 하드웨어 테스트")
        self.test_motor_btn.clicked.connect(self.test_motor_hardware)
        encoder_layout.addWidget(self.test_motor_btn)
        
        self.check_pins_btn = QPushButton("핀 상태 확인")
        self.check_pins_btn.clicked.connect(self.check_pin_status)
        encoder_layout.addWidget(self.check_pins_btn)
        
        layout.addWidget(encoder_group)
        
        # 현재 상태 그룹
        status_group = QGroupBox("현재 상태")
        status_layout = QGridLayout(status_group)
        
        status_layout.addWidget(QLabel("현재 RPM:"), 0, 0)
        self.current_rpm_label = QLabel("0.0")
        self.current_rpm_label.setStyleSheet("font-weight: bold; color: blue;")
        status_layout.addWidget(self.current_rpm_label, 0, 1)
        
        status_layout.addWidget(QLabel("현재 위치:"), 1, 0)
        self.current_position_label = QLabel("0")
        self.current_position_label.setStyleSheet("font-weight: bold; color: green;")
        status_layout.addWidget(self.current_position_label, 1, 1)
        
        status_layout.addWidget(QLabel("방향:"), 2, 0)
        self.direction_label = QLabel("정지")
        self.direction_label.setStyleSheet("font-weight: bold; color: red;")
        status_layout.addWidget(self.direction_label, 2, 1)
        
        layout.addWidget(status_group)
        
        # 상태 업데이트 타이머 (극한 최적화: 1초로 증가)
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(3000)  # 3초마다 업데이트 (10년 전 컴퓨터 최적화)
        
        return tab
        
    def create_pid_tuning_tab(self):
        """PID 튜닝 탭 생성"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # PID 파라미터 그룹
        pid_group = QGroupBox("PID 파라미터")
        pid_layout = QGridLayout(pid_group)
        
        # Kp 설정
        pid_layout.addWidget(QLabel("Kp (비례):"), 0, 0)
        self.kp_spinbox = QDoubleSpinBox()
        self.kp_spinbox.setRange(0.0, 10.0)
        self.kp_spinbox.setSingleStep(0.1)
        self.kp_spinbox.setValue(0.3)
        self.kp_spinbox.setDecimals(2)
        pid_layout.addWidget(self.kp_spinbox, 0, 1)
        
        # Ki 설정
        pid_layout.addWidget(QLabel("Ki (적분):"), 1, 0)
        self.ki_spinbox = QDoubleSpinBox()
        self.ki_spinbox.setRange(0.0, 10.0)
        self.ki_spinbox.setSingleStep(0.1)
        self.ki_spinbox.setValue(2.0)
        self.ki_spinbox.setDecimals(2)
        pid_layout.addWidget(self.ki_spinbox, 1, 1)
        
        # Kd 설정
        pid_layout.addWidget(QLabel("Kd (미분):"), 2, 0)
        self.kd_spinbox = QDoubleSpinBox()
        self.kd_spinbox.setRange(0.0, 5.0)
        self.kd_spinbox.setSingleStep(0.01)
        self.kd_spinbox.setValue(0.1)
        self.kd_spinbox.setDecimals(2)
        pid_layout.addWidget(self.kd_spinbox, 2, 1)
        
        layout.addWidget(pid_group)
        
        # 속도 제어 그룹
        speed_group = QGroupBox("속도 제어")
        speed_layout = QGridLayout(speed_group)
        
        speed_layout.addWidget(QLabel("목표 RPM:"), 0, 0)
        self.target_rpm_spinbox = QDoubleSpinBox()
        self.target_rpm_spinbox.setRange(0.0, 100.0)
        self.target_rpm_spinbox.setSingleStep(5.0)
        self.target_rpm_spinbox.setValue(50.0)
        self.target_rpm_spinbox.setDecimals(1)
        speed_layout.addWidget(self.target_rpm_spinbox, 0, 1)
        
        speed_layout.addWidget(QLabel("테스트 시간 (초):"), 1, 0)
        self.test_duration_spinbox = QSpinBox()
        self.test_duration_spinbox.setRange(1, 60)
        self.test_duration_spinbox.setValue(10)
        speed_layout.addWidget(self.test_duration_spinbox, 1, 1)
        
        # 제어 버튼들
        self.start_pid_btn = QPushButton("PID 제어 시작")
        self.start_pid_btn.clicked.connect(self.start_pid_control)
        speed_layout.addWidget(self.start_pid_btn, 2, 0)
        
        self.stop_pid_btn = QPushButton("PID 제어 중지")
        self.stop_pid_btn.clicked.connect(self.stop_pid_control)
        self.stop_pid_btn.setEnabled(False)
        speed_layout.addWidget(self.stop_pid_btn, 2, 1)
        
        layout.addWidget(speed_group)
        
        # 자동 튜닝 그룹
        auto_tune_group = QGroupBox("자동 튜닝")
        auto_tune_layout = QVBoxLayout(auto_tune_group)
        
        # 자동 튜닝 버튼들
        auto_tune_btn_layout = QHBoxLayout()
        
        self.auto_tune_btn = QPushButton("자동 PID 튜닝")
        self.auto_tune_btn.clicked.connect(self.auto_tune_pid)
        auto_tune_btn_layout.addWidget(self.auto_tune_btn)
        
        self.stop_auto_tune_btn = QPushButton("튜닝 중지")
        self.stop_auto_tune_btn.clicked.connect(self.stop_auto_tune)
        self.stop_auto_tune_btn.setEnabled(False)
        auto_tune_btn_layout.addWidget(self.stop_auto_tune_btn)
        
        auto_tune_layout.addLayout(auto_tune_btn_layout)
        
        # 프리셋 버튼들
        preset_layout = QHBoxLayout()
        
        self.preset_conservative_btn = QPushButton("보수적")
        self.preset_conservative_btn.clicked.connect(lambda: self.set_pid_preset(0.2, 1.0, 0.05))
        preset_layout.addWidget(self.preset_conservative_btn)
        
        self.preset_balanced_btn = QPushButton("균형")
        self.preset_balanced_btn.clicked.connect(lambda: self.set_pid_preset(0.3, 2.0, 0.1))
        preset_layout.addWidget(self.preset_balanced_btn)
        
        self.preset_aggressive_btn = QPushButton("적극적")
        self.preset_aggressive_btn.clicked.connect(lambda: self.set_pid_preset(0.4, 3.0, 0.15))
        preset_layout.addWidget(self.preset_aggressive_btn)
        
        auto_tune_layout.addLayout(preset_layout)
        layout.addWidget(auto_tune_group)
        
        return tab
        
    def create_position_control_tab(self):
        """위치 제어 탭 생성"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 위치 제어 그룹
        position_group = QGroupBox("위치 제어")
        position_layout = QGridLayout(position_group)
        
        position_layout.addWidget(QLabel("목표 위치:"), 0, 0)
        self.target_position_spinbox = QSpinBox()
        self.target_position_spinbox.setRange(-1000, 1000)
        self.target_position_spinbox.setValue(100)
        position_layout.addWidget(self.target_position_spinbox, 0, 1)
        
        position_layout.addWidget(QLabel("허용 오차:"), 1, 0)
        self.tolerance_spinbox = QSpinBox()
        self.tolerance_spinbox.setRange(1, 20)
        self.tolerance_spinbox.setValue(3)
        position_layout.addWidget(self.tolerance_spinbox, 1, 1)
        
        # 위치 제어 버튼들
        self.move_to_position_btn = QPushButton("위치로 이동")
        self.move_to_position_btn.clicked.connect(self.move_to_position)
        position_layout.addWidget(self.move_to_position_btn, 2, 0)
        
        self.stop_position_btn = QPushButton("정지")
        self.stop_position_btn.clicked.connect(self.stop_motor)
        position_layout.addWidget(self.stop_position_btn, 2, 1)
        
        layout.addWidget(position_group)
        
        # 위치 테스트 그룹
        test_group = QGroupBox("위치 테스트")
        test_layout = QVBoxLayout(test_group)
        
        self.test_positions_btn = QPushButton("다중 위치 테스트")
        self.test_positions_btn.clicked.connect(self.test_multiple_positions)
        test_layout.addWidget(self.test_positions_btn)
        
        layout.addWidget(test_group)
        
        return tab
        
    def create_analysis_tab(self):
        """분석 탭 생성"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 성능 분석 그룹
        analysis_group = QGroupBox("성능 분석")
        analysis_layout = QVBoxLayout(analysis_group)
        
        self.analyze_btn = QPushButton("성능 분석")
        self.analyze_btn.clicked.connect(self.analyze_performance)
        analysis_layout.addWidget(self.analyze_btn)
        
        # 분석 결과 테이블
        self.analysis_table = QTableWidget()
        self.analysis_table.setColumnCount(2)
        self.analysis_table.setHorizontalHeaderLabels(["항목", "값"])
        analysis_layout.addWidget(self.analysis_table)
        
        layout.addWidget(analysis_group)
        
        # 데이터 관리 그룹
        data_group = QGroupBox("데이터 관리")
        data_layout = QVBoxLayout(data_group)
        
        self.clear_data_btn = QPushButton("데이터 초기화")
        self.clear_data_btn.clicked.connect(self.clear_data)
        data_layout.addWidget(self.clear_data_btn)
        
        self.save_data_btn = QPushButton("데이터 저장")
        self.save_data_btn.clicked.connect(self.save_data)
        data_layout.addWidget(self.save_data_btn)
        
        layout.addWidget(data_group)
        
        return tab
        
    def create_graph_panel(self):
        """그래프 패널 생성"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # 그래프 위젯 (10년 전 컴퓨터 최적화)
        self.graph_widget = pg.PlotWidget()
        self.graph_widget.setBackground('w')
        self.graph_widget.setLabel('left', 'RPM')
        self.graph_widget.setLabel('bottom', 'Time (s)')
        # 범례와 그리드 제거로 성능 향상
        # self.graph_widget.addLegend()
        # self.graph_widget.showGrid(x=True, y=True)
        
        # 그래프 라인들 (10년 전 컴퓨터 최적화 - 더 단순한 펜 사용)
        self.target_line = self.graph_widget.plot([], [], pen=pg.mkPen('r', width=1))
        self.actual_line = self.graph_widget.plot([], [], pen=pg.mkPen('b', width=1))
        
        layout.addWidget(self.graph_widget)
        
        # 그래프 컨트롤
        control_layout = QHBoxLayout()
        
        self.auto_scale_btn = QPushButton("자동 스케일")
        self.auto_scale_btn.clicked.connect(self.auto_scale_graph)
        control_layout.addWidget(self.auto_scale_btn)
        
        self.clear_graph_btn = QPushButton("그래프 초기화")
        self.clear_graph_btn.clicked.connect(self.clear_graph)
        control_layout.addWidget(self.clear_graph_btn)
        
        layout.addLayout(control_layout)
        
        return panel
        
    def setup_connections(self):
        """시그널 연결"""
        pass
        
    def log_message(self, message):
        """로그 메시지 출력 (10년 전 컴퓨터 최적화)"""
        # 로그 출력 빈도 제한
        if not hasattr(self, '_last_log_time'):
            self._last_log_time = 0
        
        current_time = time.time()
        if current_time - self._last_log_time < 0.5:  # 0.5초마다만 로그 출력
            return
            
        self._last_log_time = current_time
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        
        # 로그가 너무 많아지면 오래된 것 삭제 (10년 전 컴퓨터 최적화)
        if self.log_text.document().blockCount() > 20:
            cursor = self.log_text.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.movePosition(cursor.Down, cursor.KeepAnchor, 5)
            cursor.removeSelectedText()
        
        # 스크롤은 필요할 때만 (10년 전 컴퓨터 최적화)
        if self.log_text.verticalScrollBar().value() >= self.log_text.verticalScrollBar().maximum() - 5:
            self.log_text.verticalScrollBar().setValue(
                self.log_text.verticalScrollBar().maximum()
            )
        
    def update_status(self):
        """상태 업데이트 (극한 최적화)"""
        try:
            # 연결 상태 확인 후에만 데이터 요청
            if not self.motor.connected:
                return
            
            # 더 큰 변화가 있을 때만 업데이트 (10년 전 컴퓨터 최적화)
            current_rpm = self.motor.get_current_rpm()
            current_pos = self.motor.get_position()
            
            # 값이 크게 변경되었을 때만 UI 업데이트 (임계값 증가)
            rpm_changed = not hasattr(self, '_last_rpm') or abs(current_rpm - self._last_rpm) > 2.0
            pos_changed = not hasattr(self, '_last_pos') or abs(current_pos - self._last_pos) > 10
            
            if not (rpm_changed or pos_changed):
                return
                
            self.current_rpm_label.setText(f"{current_rpm:.1f}")
            self.current_position_label.setText(str(current_pos))
            
            # 방향은 RPM이 0이 아닐 때만 계산
            if current_rpm > 0:
                direction = self.motor.get_direction()
                self.direction_label.setText("정방향" if direction else "역방향")
            else:
                self.direction_label.setText("정지")
            
            # 마지막 값 저장
            self._last_rpm = current_rpm
            self._last_pos = current_pos
            
        except Exception:
            # 오류 발생 시 아무것도 하지 않음 (10년 전 컴퓨터 최적화)
            pass
            
    def update_data(self, timestamp, target_rpm, actual_rpm, kp, ki, kd, position):
        """데이터 업데이트 (10년 전 컴퓨터 최적화)"""
        # 데이터 추가
        self.data_history.append({
            'time': timestamp,
            'target_rpm': target_rpm,
            'actual_rpm': actual_rpm,
            'kp': kp,
            'ki': ki,
            'kd': kd,
            'position': position
        })
        
        # 그래프 업데이트 (최근 10개 데이터만 - 10년 전 컴퓨터 최적화)
        if len(self.data_history) > 10:
            self.data_history = self.data_history[-10:]
        
        # 그래프 업데이트는 별도로 처리 (빈도 제한)
        self.update_graph()
        
    def update_graph(self):
        """그래프 업데이트 (10년 전 컴퓨터 최적화)"""
        if not self.data_history:
            return
        
        # 데이터가 충분할 때만 업데이트 (10년 전 컴퓨터 최적화)
        if len(self.data_history) < 5:
            return
        
        # 그래프 업데이트 빈도 제한
        if not hasattr(self, '_last_graph_update'):
            self._last_graph_update = 0
            
        current_time = time.time()
        if current_time - self._last_graph_update < 1.0:  # 1초마다만 그래프 업데이트
            return
            
        self._last_graph_update = current_time
            
        times = [d['time'] for d in self.data_history]
        target_rpms = [d['target_rpm'] for d in self.data_history]
        actual_rpms = [d['actual_rpm'] for d in self.data_history]
        
        # 그래프 업데이트 (배치 처리)
        self.target_line.setData(times, target_rpms)
        self.actual_line.setData(times, actual_rpms)
        
    def auto_scale_graph(self):
        """그래프 자동 스케일 (10년 전 컴퓨터 최적화)"""
        # 자동 스케일링도 빈도 제한
        if not hasattr(self, '_last_scale_time'):
            self._last_scale_time = 0
            
        current_time = time.time()
        if current_time - self._last_scale_time < 2.0:  # 2초마다만 스케일링
            return
            
        self._last_scale_time = current_time
        self.graph_widget.enableAutoRange()
        
    def clear_graph(self):
        """그래프 초기화 (10년 전 컴퓨터 최적화)"""
        self.data_history.clear()
        self.target_line.setData([], [])
        self.actual_line.setData([], [])
        
        # 메모리 정리
        import gc
        gc.collect()
        
        self.log_message("그래프 초기화됨")
        
    def forward_motor(self):
        """정방향 모터 제어 (10년 전 컴퓨터 최적화)"""
        speed = self.speed_spinbox.value()
        self.log_message(f"정방향 {speed}% 명령 전송 중...")
        
        # 연결 상태 확인
        if not self.motor.connected:
            self.log_message("오류: UART 연결되지 않음")
            return
        
        # UI 반응성을 위해 별도 스레드에서 실행
        threading.Thread(target=self._forward_motor_thread, args=(speed,), daemon=True).start()
        
    def _forward_motor_thread(self, speed):
        """정방향 모터 제어 스레드"""
        # 명령 전송
        response = self.motor.forward(speed)
        if response:
            self.log_message(f"정방향 {speed}% 명령 전송 성공")
            self.log_message(f"ESP32 응답: {response}")
        else:
            self.log_message(f"정방향 {speed}% 명령 전송 실패 - 응답 없음")
        
    def backward_motor(self):
        """역방향 모터 제어 (10년 전 컴퓨터 최적화)"""
        speed = self.speed_spinbox.value()
        self.log_message(f"역방향 {speed}% 시작")
        
        # UI 반응성을 위해 별도 스레드에서 실행
        threading.Thread(target=self._backward_motor_thread, args=(speed,), daemon=True).start()
        
    def _backward_motor_thread(self, speed):
        """역방향 모터 제어 스레드"""
        self.motor.backward(speed)
        
    def stop_motor(self):
        """모터 정지 (10년 전 컴퓨터 최적화)"""
        self.log_message("모터 정지")
        
        # UI 반응성을 위해 별도 스레드에서 실행
        threading.Thread(target=self._stop_motor_thread, daemon=True).start()
        
    def _stop_motor_thread(self):
        """모터 정지 스레드"""
        self.motor.stop()
        
    def reset_position(self):
        """위치 리셋"""
        self.motor.reset_position()
        self.log_message("위치 리셋됨")
        
    def test_encoder(self):
        """엔코더 테스트"""
        self.log_message("엔코더 테스트 시작 - 모터를 수동으로 회전시켜보세요")
        # 실제 테스트는 별도 스레드에서 실행
        threading.Thread(target=self._encoder_test_thread, daemon=True).start()
    
    def test_motor_hardware(self):
        """모터 하드웨어 테스트"""
        if not self.motor.connected:
            self.log_message("오류: UART 연결되지 않음")
            return
            
        self.log_message("모터 하드웨어 테스트 시작")
        response = self.motor.send_command('test_motor')
        if response:
            self.log_message("모터 하드웨어 테스트 완료")
            self.log_message(f"ESP32 응답: {response}")
        else:
            self.log_message("모터 하드웨어 테스트 실패 - 응답 없음")
    
    def check_pin_status(self):
        """핀 상태 확인"""
        if not self.motor.connected:
            self.log_message("오류: UART 연결되지 않음")
            return
            
        self.log_message("핀 상태 확인 중")
        response = self.motor.send_command('check_pins')
        if response and 'data' in response:
            pin_data = response['data']
            self.log_message("=== 핀 상태 ===")
            for pin_name, status in pin_data.items():
                self.log_message(f"{pin_name}: {status}")
            self.log_message("===============")
        else:
            self.log_message("핀 상태 확인 실패 - 응답 없음")
        
    def _encoder_test_thread(self):
        """엔코더 테스트 스레드"""
        initial_pos = self.motor.get_position()
        start_time = time.time()
        
        for i in range(50):  # 5초간 테스트
            current_pos = self.motor.get_position()
            elapsed = time.time() - start_time
            self.log_message(f"시간: {elapsed:.1f}s, 펄스: {current_pos}")
            time.sleep(0.1)
            
        final_pos = self.motor.get_position()
        total_pulses = final_pos - initial_pos
        self.log_message(f"엔코더 테스트 완료 - 총 펄스: {total_pulses}")
        
    def start_pid_control(self):
        """PID 제어 시작"""
        if self.is_testing:
            return
            
        kp = self.kp_spinbox.value()
        ki = self.ki_spinbox.value()
        kd = self.kd_spinbox.value()
        target_rpm = self.target_rpm_spinbox.value()
        test_duration = self.test_duration_spinbox.value()
        
        # PID 파라미터 설정
        self.motor.set_speed_pid_parameters(kp=kp, ki=ki, kd=kd)
        self.motor.reset_position()
        
        # 데이터 로깅 시작
        self.data_logger.start_logging(target_rpm, kp, ki, kd)
        
        # UI 상태 변경
        self.is_testing = True
        self.start_pid_btn.setEnabled(False)
        self.stop_pid_btn.setEnabled(True)
        
        self.log_message(f"PID 제어 시작 - Kp:{kp}, Ki:{ki}, Kd:{kd}, 목표:{target_rpm}RPM")
        
        # 테스트 종료 타이머
        self.test_timer = QTimer()
        self.test_timer.timeout.connect(self.stop_pid_control)
        self.test_timer.setSingleShot(True)
        self.test_timer.start(test_duration * 1000)
        
    def stop_pid_control(self):
        """PID 제어 중지"""
        if not self.is_testing:
            return
            
        # 데이터 로깅 중지
        self.data_logger.stop_logging()
        
        # 모터 정지
        self.motor.stop()
        
        # UI 상태 변경
        self.is_testing = False
        self.start_pid_btn.setEnabled(True)
        self.stop_pid_btn.setEnabled(False)
        
        self.log_message("PID 제어 중지")
        
    def set_pid_preset(self, kp, ki, kd):
        """PID 프리셋 설정"""
        self.kp_spinbox.setValue(kp)
        self.ki_spinbox.setValue(ki)
        self.kd_spinbox.setValue(kd)
        self.log_message(f"PID 프리셋 적용 - Kp:{kp}, Ki:{ki}, Kd:{kd}")
        
    def auto_tune_pid(self):
        """자동 PID 튜닝"""
        if self.is_auto_tuning:
            self.log_message("자동 튜닝이 이미 실행 중입니다")
            return
            
        self.log_message("자동 PID 튜닝 시작")
        self.is_auto_tuning = True
        
        # UI 상태 변경
        self.auto_tune_btn.setEnabled(False)
        self.stop_auto_tune_btn.setEnabled(True)
        
        # 실제 튜닝은 별도 스레드에서 실행
        threading.Thread(target=self._auto_tune_thread, daemon=True).start()
    
    def stop_auto_tune(self):
        """자동 튜닝 중지"""
        if not self.is_auto_tuning:
            self.log_message("자동 튜닝이 실행 중이 아닙니다")
            return
            
        self.log_message("자동 PID 튜닝 중지 요청")
        self.is_auto_tuning = False
        
        # 모터 정지
        self.motor.stop()
        
        # UI 상태 복원
        self.auto_tune_btn.setEnabled(True)
        self.stop_auto_tune_btn.setEnabled(False)
        
        self.log_message("자동 PID 튜닝 중지됨")
        
    def _auto_tune_thread(self):
        """자동 튜닝 스레드"""
        target_rpm = self.target_rpm_spinbox.value()
        
        # 테스트할 파라미터 조합
        test_configs = [
            {"name": "보수적", "kp": 0.2, "ki": 1.0, "kd": 0.05},
            {"name": "균형", "kp": 0.3, "ki": 2.0, "kd": 0.1},
            {"name": "적극적", "kp": 0.4, "ki": 3.0, "kd": 0.15},
        ]
        
        best_config = None
        best_score = float('inf')
        
        for config in test_configs:
            # 중지 요청 확인
            if not self.is_auto_tuning:
                self.log_message("자동 튜닝이 중지되었습니다")
                break
                
            self.log_message(f"테스트: {config['name']}")
            
            # 테스트 실행
            self.motor.set_speed_pid_parameters(
                kp=config['kp'], ki=config['ki'], kd=config['kd']
            )
            self.motor.reset_position()
            
            # 5초간 테스트 (중지 요청 확인 포함)
            start_time = time.time()
            rpm_measurements = []
            
            while time.time() - start_time < 5 and self.is_auto_tuning:
                current_rpm = self.motor.control_speed_pid(target_rpm)
                rpm_measurements.append(current_rpm)
                time.sleep(0.1)
                
            self.motor.stop()
            
            # 중지 요청 확인
            if not self.is_auto_tuning:
                self.log_message("자동 튜닝이 중지되었습니다")
                break
            
            # 성능 평가
            if rpm_measurements:
                avg_rpm = sum(rpm_measurements) / len(rpm_measurements)
                error = abs(target_rpm - avg_rpm)
                variance = sum((rpm - avg_rpm)**2 for rpm in rpm_measurements) / len(rpm_measurements)
                std_dev = math.sqrt(variance)
                
                score = error + std_dev  # 오차 + 표준편차
                
                self.log_message(f"{config['name']}: 오차={error:.1f}, 표준편차={std_dev:.2f}, 점수={score:.2f}")
                
                if score < best_score:
                    best_score = score
                    best_config = config
                    
            # 중지 요청 확인 후 대기
            if self.is_auto_tuning:
                time.sleep(1)
            
        # UI 상태 복원
        self.auto_tune_btn.setEnabled(True)
        self.stop_auto_tune_btn.setEnabled(False)
        
        # 튜닝이 완료된 경우 결과 적용
        if best_config and self.is_auto_tuning:
            self.log_message(f"최적 설정: {best_config['name']} - Kp:{best_config['kp']}, Ki:{best_config['ki']}, Kd:{best_config['kd']}")
            # UI에 최적 설정 적용
            self.kp_spinbox.setValue(best_config['kp'])
            self.ki_spinbox.setValue(best_config['ki'])
            self.kd_spinbox.setValue(best_config['kd'])
            self.log_message("자동 PID 튜닝 완료")
        else:
            self.log_message("자동 튜닝이 중지되었거나 실패했습니다")
            
        # 상태 초기화
        self.is_auto_tuning = False
            
    def move_to_position(self):
        """위치로 이동"""
        target_pos = self.target_position_spinbox.value()
        tolerance = self.tolerance_spinbox.value()
        
        self.log_message(f"위치 {target_pos}로 이동 시작 (허용 오차: {tolerance})")
        
        # 위치 제어 실행
        threading.Thread(
            target=self._position_control_thread, 
            args=(target_pos, tolerance), 
            daemon=True
        ).start()
        
    def _position_control_thread(self, target_pos, tolerance):
        """위치 제어 스레드"""
        self.motor.reset_position()
        start_time = time.time()
        
        while time.time() - start_time < 10:  # 10초 타임아웃
            current_pos = self.motor.control_position_pid(target_pos)
            error = abs(target_pos - current_pos)
            
            if error <= tolerance:
                self.log_message(f"목표 도달! 위치: {current_pos}, 오차: {error}")
                break
                
            time.sleep(0.1)
            
        self.motor.stop()
        
    def test_multiple_positions(self):
        """다중 위치 테스트"""
        positions = [50, 100, -50, 200, 0]
        self.log_message("다중 위치 테스트 시작")
        
        threading.Thread(target=self._multi_position_test_thread, daemon=True).start()
        
    def _multi_position_test_thread(self):
        """다중 위치 테스트 스레드"""
        positions = [50, 100, -50, 200, 0]
        
        for i, pos in enumerate(positions):
            self.log_message(f"테스트 {i+1}/5: 위치 {pos}")
            self._position_control_thread(pos, 3)
            time.sleep(1)
            
        self.log_message("다중 위치 테스트 완료")
        
    def analyze_performance(self):
        """성능 분석"""
        if not self.data_history:
            QMessageBox.warning(self, "경고", "분석할 데이터가 없습니다.")
            return
            
        # RPM 데이터 분석
        target_rpms = [d['target_rpm'] for d in self.data_history if d['target_rpm'] > 0]
        actual_rpms = [d['actual_rpm'] for d in self.data_history if d['actual_rpm'] > 0]
        
        if not actual_rpms:
            QMessageBox.warning(self, "경고", "실제 RPM 데이터가 없습니다.")
            return
            
        # 통계 계산
        avg_target = sum(target_rpms) / len(target_rpms) if target_rpms else 0
        avg_actual = sum(actual_rpms) / len(actual_rpms)
        max_actual = max(actual_rpms)
        min_actual = min(actual_rpms)
        
        # 오차 계산
        errors = [abs(t - a) for t, a in zip(target_rpms, actual_rpms)]
        avg_error = sum(errors) / len(errors)
        max_error = max(errors)
        
        # 표준편차 계산
        variance = sum((rpm - avg_actual)**2 for rpm in actual_rpms) / len(actual_rpms)
        std_dev = math.sqrt(variance)
        
        # 성공률 계산
        success_count = sum(1 for error in errors if error <= 2.0)
        success_rate = (success_count / len(errors)) * 100
        
        # 결과 테이블에 표시
        results = [
            ("목표 RPM", f"{avg_target:.1f}"),
            ("실제 평균 RPM", f"{avg_actual:.1f}"),
            ("실제 최대 RPM", f"{max_actual:.1f}"),
            ("실제 최소 RPM", f"{min_actual:.1f}"),
            ("평균 오차", f"{avg_error:.1f} RPM"),
            ("최대 오차", f"{max_error:.1f} RPM"),
            ("표준편차", f"{std_dev:.2f} RPM"),
            ("성공률", f"{success_rate:.1f}%")
        ]
        
        self.analysis_table.setRowCount(len(results))
        for i, (item, value) in enumerate(results):
            self.analysis_table.setItem(i, 0, QTableWidgetItem(item))
            self.analysis_table.setItem(i, 1, QTableWidgetItem(value))
            
        self.log_message("성능 분석 완료")
        
    def clear_data(self):
        """데이터 초기화 (10년 전 컴퓨터 최적화)"""
        self.data_history.clear()
        self.clear_graph()
        self.analysis_table.setRowCount(0)
        
        # 메모리 정리
        import gc
        gc.collect()
        
        self.log_message("모든 데이터 초기화됨")
        
    def save_data(self):
        """데이터 저장"""
        if not self.data_history:
            QMessageBox.warning(self, "경고", "저장할 데이터가 없습니다.")
            return
            
        filename, _ = QFileDialog.getSaveFileName(
            self, "데이터 저장", "motor_data.txt", "Text Files (*.txt)"
        )
        
        if filename:
            try:
                with open(filename, 'w') as f:
                    f.write("Time,Target_RPM,Actual_RPM,Kp,Ki,Kd,Position\n")
                    for data in self.data_history:
                        f.write(f"{data['time']:.1f},{data['target_rpm']:.1f},"
                               f"{data['actual_rpm']:.1f},{data['kp']:.2f},"
                               f"{data['ki']:.2f},{data['kd']:.2f},{data['position']}\n")
                self.log_message(f"데이터 저장 완료: {filename}")
            except Exception as e:
                QMessageBox.critical(self, "오류", f"데이터 저장 실패: {e}")
                
    def closeEvent(self, event):
        """프로그램 종료 시 정리"""
        try:
            self.motor.stop()
            if hasattr(self, 'data_logger'):
                self.data_logger.stop_logging()
            self.motor.disconnect()
        except:
            pass
        event.accept()


def main():
    """메인 함수"""
    app = QApplication(sys.argv)
    
    # 스타일 설정
    app.setStyle('Fusion')
    
    # 다크 테마
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(25, 25, 25))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ToolTipBase, Qt.white)
    palette.setColor(QPalette.ToolTipText, Qt.white)
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, Qt.white)
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(palette)
    
    window = MotorDeveloperGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
