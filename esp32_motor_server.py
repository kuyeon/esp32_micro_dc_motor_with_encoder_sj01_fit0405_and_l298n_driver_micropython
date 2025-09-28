"""
ESP32 MicroPython 모터 서버
UART를 통해 PC와 통신하여 모터 제어
FIT0405 모터 + SJ01 엔코더 + L298N 드라이버용
"""

import json
import time
from machine import Pin, PWM, UART
from fit0405_motor import FIT0405Motor

class MotorServer:
    def __init__(self, uart_port=1, tx_pin=17, rx_pin=16, baudrate=115200):
        """모터 서버 초기화"""
        # UART 설정
        self.uart = UART(uart_port, baudrate=baudrate, tx=tx_pin, rx=rx_pin)
        
        # 모터 초기화
        self.motor = FIT0405Motor()
        
        # 명령 처리 상태
        self.running = True
        
        print("모터 서버 시작됨")
        print(f"UART: {baudrate} baud, TX={tx_pin}, RX={rx_pin}")
        
    def send_response(self, success=True, data=None, message=""):
        """응답 전송"""
        response = {
            'success': success,
            'data': data,
            'message': message,
            'timestamp': time.time()
        }
        
        json_str = json.dumps(response) + '\n'
        self.uart.write(json_str.encode())
        
    def process_command(self, command_data):
        """명령 처리"""
        try:
            command = command_data.get('command')
            data = command_data.get('data', {})
            
            if command == 'forward':
                speed = data.get('speed', 50)
                self.motor.forward(speed)
                self.send_response(True, {'speed': speed}, "정방향 회전 시작")
                
            elif command == 'backward':
                speed = data.get('speed', 50)
                self.motor.backward(speed)
                self.send_response(True, {'speed': speed}, "역방향 회전 시작")
                
            elif command == 'stop':
                self.motor.stop()
                self.send_response(True, message="모터 정지")
                
            elif command == 'reset_position':
                self.motor.reset_position()
                self.send_response(True, message="위치 리셋됨")
                
            elif command == 'get_position':
                position = self.motor.get_position()
                self.send_response(True, {'position': position})
                
            elif command == 'get_rpm':
                rpm = self.motor.get_current_rpm()
                self.send_response(True, {'rpm': rpm})
                
            elif command == 'get_direction':
                direction = self.motor.get_direction()
                self.send_response(True, {'direction': direction})
                
            elif command == 'set_speed_pid':
                kp = data.get('kp', 0.3)
                ki = data.get('ki', 2.0)
                kd = data.get('kd', 0.1)
                self.motor.set_speed_pid_parameters(kp=kp, ki=ki, kd=kd)
                self.send_response(True, {'kp': kp, 'ki': ki, 'kd': kd}, "속도 PID 파라미터 설정됨")
                
            elif command == 'set_position_pid':
                kp = data.get('kp', 0.1)
                ki = data.get('ki', 0.1)
                kd = data.get('kd', 0.1)
                self.motor.set_position_pid_parameters(kp=kp, ki=ki, kd=kd)
                self.send_response(True, {'kp': kp, 'ki': ki, 'kd': kd}, "위치 PID 파라미터 설정됨")
                
            elif command == 'control_speed_pid':
                target_rpm = data.get('target_rpm', 50.0)
                current_rpm = self.motor.control_speed_pid(target_rpm)
                self.send_response(True, {'current_rpm': current_rpm, 'target_rpm': target_rpm})
                
            elif command == 'control_position_pid':
                target_position = data.get('target_position', 100)
                current_position = self.motor.control_position_pid(target_position)
                self.send_response(True, {'current_position': current_position, 'target_position': target_position})
                
            elif command == 'get_status':
                status = {
                    'position': self.motor.get_position(),
                    'rpm': self.motor.get_current_rpm(),
                    'direction': self.motor.get_direction()
                }
                self.send_response(True, status)
                
            elif command == 'ping':
                self.send_response(True, {'pong': True}, "연결 확인됨")
                
            else:
                self.send_response(False, message=f"알 수 없는 명령: {command}")
                
        except Exception as e:
            self.send_response(False, message=f"명령 처리 오류: {str(e)}")
            
    def run(self):
        """메인 루프"""
        print("명령 대기 중...")
        
        while self.running:
            try:
                # UART에서 데이터 읽기
                if self.uart.any():
                    line = self.uart.readline()
                    if line:
                        try:
                            # JSON 파싱
                            command_data = json.loads(line.decode().strip())
                            print(f"수신된 명령: {command_data}")
                            
                            # 명령 처리
                            self.process_command(command_data)
                            
                        except json.JSONDecodeError as e:
                            print(f"JSON 파싱 오류: {e}")
                            self.send_response(False, message="잘못된 JSON 형식")
                            
                        except Exception as e:
                            print(f"명령 처리 오류: {e}")
                            self.send_response(False, message=f"처리 오류: {str(e)}")
                
                # 짧은 대기
                time.sleep(0.01)
                
            except KeyboardInterrupt:
                print("서버 종료 중...")
                self.running = False
                break
            except Exception as e:
                print(f"메인 루프 오류: {e}")
                time.sleep(0.1)
                
        # 정리
        self.motor.stop()
        print("모터 서버 종료됨")

def main():
    """메인 함수"""
    # 서버 생성 및 실행
    server = MotorServer()
    server.run()

if __name__ == "__main__":
    main()
