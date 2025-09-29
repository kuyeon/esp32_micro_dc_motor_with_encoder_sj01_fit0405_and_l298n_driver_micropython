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
    def __init__(self, uart_port=1, tx_pin=43, rx_pin=44, baudrate=115200):
        """모터 서버 초기화"""
        # UART 설정 (ESP32 USB 포트: COM1, 디버깅 터미널 포트: COM3)
        self.uart = UART(uart_port, baudrate=baudrate, tx=tx_pin, rx=rx_pin, timeout=1000)
        
        # UART 버퍼 초기화 (null 바이트 문제 해결)
        self.uart.flush()
        
        # 메시지 버퍼링을 위한 변수들
        self.message_buffer = b''
        self.max_buffer_size = 1024  # 최대 버퍼 크기
        
        # 모터 초기화
        self.motor = FIT0405Motor()
        
        # 명령 처리 상태
        self.running = True
        
        print("모터 서버 시작됨")
        print(f"UART: {baudrate} baud, TX={tx_pin}, RX={rx_pin}")
        print("포트 정보: ESP32 USB 포트=COM1, 디버깅 터미널 포트=COM3")
        print(f"UART0 핀 설정: TX=GPIO{tx_pin}, RX=GPIO{rx_pin}")
        print("주의: UART0은 ESP32 하드웨어 시리얼 포트입니다")
        
        # UART 연결 테스트
        try:
            # 버퍼 클리어
            self.uart.flush()
            time.sleep(0.1)
            
            # 테스트 데이터 전송
            test_data = b"UART_TEST\n"
            bytes_written = self.uart.write(test_data)
            print(f"UART 테스트 데이터 전송: {bytes_written} 바이트")
            
            # 초기화 후 잠시 대기
            time.sleep(0.2)
            
            # UART 상태 확인
            print(f"UART 상태: any()={self.uart.any()}")
            print("UART 초기화 테스트 완료")
            
        except Exception as e:
            print(f"UART 초기화 오류: {e}")
        
        # 연결 테스트
        self.send_response(True, {'status': 'ready'}, "서버 준비 완료")
        
    def process_complete_messages(self):
        """완전한 메시지 처리"""
        try:
            # 버퍼를 문자열로 변환
            buffer_str = self.message_buffer.decode('utf-8')
            print(f"현재 버퍼 내용: '{buffer_str}'")
            
            # 줄바꿈으로 메시지 분리
            lines = buffer_str.split('\n')
            
            # 마지막 줄이 완전하지 않을 수 있으므로 제외
            complete_lines = lines[:-1]
            incomplete_line = lines[-1] if lines else ""
            
            # 완전한 메시지들 처리
            for line in complete_lines:
                line = line.strip()
                if line and len(line) > 2:  # 최소 길이 체크
                    try:
                        # JSON 파싱
                        command_data = json.loads(line)
                        print(f"완전한 메시지 파싱 성공: {command_data}")
                        
                        # 명령 처리
                        self.process_command(command_data)
                    except Exception as json_error:
                        print(f"JSON 파싱 실패: {json_error}")
                        print(f"문제가 된 데이터: '{line}'")
                        self.send_response(False, message=f"JSON 파싱 실패: {str(json_error)}")
                elif line:
                    print(f"너무 짧은 데이터: '{line}'")
            
            # 버퍼 업데이트 (완전하지 않은 마지막 줄만 유지)
            if incomplete_line:
                self.message_buffer = incomplete_line.encode('utf-8')
                print(f"불완전한 메시지 버퍼에 유지: '{incomplete_line}'")
            else:
                self.message_buffer = b''
                print("버퍼 초기화됨")
                
        except UnicodeDecodeError as e:
            print(f"문자 인코딩 오류: {e}")
            print(f"문제가 된 버퍼: {self.message_buffer}")
            # 인코딩 오류 시 버퍼 초기화
            self.message_buffer = b''
        except Exception as e:
            print(f"메시지 처리 오류: {e}")
            # 오류 시 버퍼 초기화
            self.message_buffer = b''

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
                print(f"정방향 명령 수신 - 속도: {speed}%")
                self.motor.forward(speed)
                print(f"모터 정방향 명령 실행 완료")
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
                print(f"속도 PID 파라미터 설정 - Kp:{kp}, Ki:{ki}, Kd:{kd}")
                self.motor.set_speed_pid_parameters(kp=kp, ki=ki, kd=kd)
                print("속도 PID 파라미터 설정 완료")
                self.send_response(True, {'kp': kp, 'ki': ki, 'kd': kd}, "속도 PID 파라미터 설정됨")
                
            elif command == 'set_position_pid':
                kp = data.get('kp', 0.1)
                ki = data.get('ki', 0.1)
                kd = data.get('kd', 0.1)
                self.motor.set_position_pid_parameters(kp=kp, ki=ki, kd=kd)
                self.send_response(True, {'kp': kp, 'ki': ki, 'kd': kd}, "위치 PID 파라미터 설정됨")
                
            elif command == 'control_speed_pid':
                target_rpm = data.get('target_rpm', 50.0)
                print(f"PID 속도 제어 명령 수신 - 목표 RPM: {target_rpm}")
                current_rpm = self.motor.control_speed_pid(target_rpm)
                print(f"PID 속도 제어 완료 - 현재 RPM: {current_rpm}")
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
                print("Ping 명령 수신됨 - Pong 응답 전송")
                self.send_response(True, {'pong': True}, "연결 확인됨")
                
            elif command == 'test_motor':
                print("모터 하드웨어 테스트 시작")
                self.test_motor_hardware()
                self.send_response(True, message="모터 하드웨어 테스트 완료")
                
            elif command == 'check_pins':
                print("핀 상태 확인")
                pin_status = self.check_pin_status()
                self.send_response(True, pin_status, "핀 상태 확인 완료")
                
            else:
                self.send_response(False, message=f"알 수 없는 명령: {command}")
                
        except Exception as e:
            self.send_response(False, message=f"명령 처리 오류: {str(e)}")
    
    def test_motor_hardware(self):
        """모터 하드웨어 테스트"""
        print("=== 모터 하드웨어 테스트 시작 ===")
        
        # 1. 핀 상태 확인
        print("1. 핀 상태 확인:")
        pin_status = self.check_pin_status()
        for pin_name, status in pin_status.items():
            print(f"   {pin_name}: {status}")
        
        # 2. PWM 테스트
        print("2. PWM 테스트:")
        try:
            # Enable 핀 PWM 테스트
            enable_pin = self.motor.motor.pwm_enable
            print(f"   Enable 핀 PWM 주파수: {enable_pin.freq()}Hz")
            print(f"   Enable 핀 현재 duty: {enable_pin.duty()}")
            
            # PWM duty 테스트 (10% -> 50% -> 90%)
            for duty_test in [100, 500, 900]:
                enable_pin.duty(duty_test)
                print(f"   PWM duty {duty_test} 설정 완료")
                time.sleep(0.5)
            
            # PWM 정지
            enable_pin.duty(0)
            print("   PWM 정지 완료")
            
        except Exception as e:
            print(f"   PWM 테스트 오류: {e}")
        
        # 3. 방향 핀 테스트
        print("3. 방향 핀 테스트:")
        try:
            in1 = self.motor.motor.in1
            in2 = self.motor.motor.in2
            
            # 정방향 테스트
            in1.value(1)
            in2.value(0)
            print("   정방향 핀 설정: IN1=1, IN2=0")
            time.sleep(0.5)
            
            # 역방향 테스트
            in1.value(0)
            in2.value(1)
            print("   역방향 핀 설정: IN1=0, IN2=1")
            time.sleep(0.5)
            
            # 정지
            in1.value(0)
            in2.value(0)
            print("   정지 핀 설정: IN1=0, IN2=0")
            
        except Exception as e:
            print(f"   방향 핀 테스트 오류: {e}")
        
        # 4. 통합 모터 테스트
        print("4. 통합 모터 테스트:")
        try:
            # 낮은 속도로 테스트
            print("   정방향 10% 속도 테스트")
            self.motor.forward(10)
            time.sleep(1)
            
            print("   정지")
            self.motor.stop()
            time.sleep(0.5)
            
            print("   역방향 10% 속도 테스트")
            self.motor.backward(10)
            time.sleep(1)
            
            print("   정지")
            self.motor.stop()
            
        except Exception as e:
            print(f"   통합 모터 테스트 오류: {e}")
        
        print("=== 모터 하드웨어 테스트 완료 ===")
    
    def check_pin_status(self):
        """핀 상태 확인"""
        pin_status = {}
        
        try:
            # 모터 드라이버 핀들
            in1 = self.motor.motor.in1
            in2 = self.motor.motor.in2
            enable_pwm = self.motor.motor.pwm_enable
            
            pin_status['IN1'] = in1.value()
            pin_status['IN2'] = in2.value()
            pin_status['Enable_PWM_duty'] = enable_pwm.duty()
            pin_status['Enable_PWM_freq'] = enable_pwm.freq()
            
            # 엔코더 핀들
            encoder_a = self.motor.encoder.pin_a
            encoder_b = self.motor.encoder.pin_b
            
            pin_status['Encoder_A'] = encoder_a.value()
            pin_status['Encoder_B'] = encoder_b.value()
            pin_status['Pulse_Count'] = self.motor.encoder.pulse_count
            
        except Exception as e:
            pin_status['Error'] = str(e)
        
        return pin_status
            
    def run(self):
        """메인 루프"""
        print("명령 대기 중...")
        print("ESP32에서 UART 데이터 수신 대기 중...")
        
        loop_count = 0
        while self.running:
            try:
                loop_count += 1
                
                # 1000번 루프마다 상태 출력 (디버깅)
                if loop_count % 1000 == 0:
                    print(f"UART 대기 중... (루프: {loop_count})")
                
                # UART에서 데이터 읽기 (메시지 버퍼링)
                if self.uart.any():
                    try:
                        # 사용 가능한 바이트 수 확인
                        available_bytes = self.uart.any()
                        print(f"UART에서 {available_bytes} 바이트 수신됨")
                        
                        # 모든 데이터 읽기
                        raw_data = self.uart.read(available_bytes)
                        print(f"원시 데이터: {raw_data}")
                        
                        if raw_data:
                            # null 바이트 필터링
                            filtered_data = raw_data.replace(b'\x00', b'')
                            
                            if not filtered_data:
                                print("null 바이트만 수신됨 - 무시")
                                continue
                            
                            # 버퍼에 데이터 추가
                            self.message_buffer += filtered_data
                            
                            # 버퍼 크기 제한
                            if len(self.message_buffer) > self.max_buffer_size:
                                print("버퍼 오버플로우 - 버퍼 초기화")
                                self.message_buffer = b''
                                continue
                            
                            # 완전한 메시지 처리
                            self.process_complete_messages()
                        else:
                            print("UART에서 빈 데이터 수신됨")
                                
                    except (ValueError, TypeError) as e:
                        print(f"JSON 파싱 오류: {e}")
                        self.send_response(False, message="잘못된 JSON 형식")
                        
                    except UnicodeDecodeError as e:
                        print(f"문자 인코딩 오류: {e}")
                        self.send_response(False, message="인코딩 오류")
                        
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
