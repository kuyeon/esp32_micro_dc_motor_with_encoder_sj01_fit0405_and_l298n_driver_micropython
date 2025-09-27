"""
ESP32용 마이크로 DC 모터 엔코더 및 L298N 드라이버 라이브러리
SJ01 FIT0405 엔코더 모터와 L298N 드라이버를 제어합니다.

연결:
- 엔코더 Phase A: GPIO 18
- 엔코더 Phase B: GPIO 17 (인터럽트 핀)
- L298N IN1: GPIO 4
- L298N IN2: GPIO 5
"""

import machine
import time
from machine import Pin, PWM

class Encoder:
    """엔코더 클래스 - 펄스 카운팅 및 방향 감지"""
    
    def __init__(self, pin_a=18, pin_b=17):
        """
        엔코더 초기화
        
        Args:
            pin_a (int): Phase A 핀 번호 (기본값: 18)
            pin_b (int): Phase B 핀 번호 (기본값: 17)
        """
        self.pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        
        # 펄스 카운터 및 방향
        self.pulse_count = 0
        self.direction = True  # True: 정방향, False: 역방향
        self.last_state_a = self.pin_a.value()
        
        # 인터럽트 설정
        self.pin_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, 
                      handler=self._encoder_interrupt)
        
        print("엔코더 초기화 완료")
        print(f"Phase A: GPIO {pin_a}")
        print(f"Phase B: GPIO {pin_b}")
    
    def _encoder_interrupt(self, pin):
        """엔코더 인터럽트 핸들러"""
        current_state_a = self.pin_a.value()
        
        # Phase A의 상승 엣지에서만 처리
        if self.last_state_a == 0 and current_state_a == 1:
            state_b = self.pin_b.value()
            
            if state_b == 0 and self.direction:
                self.direction = False  # 역방향
            elif state_b == 1 and not self.direction:
                self.direction = True   # 정방향
        
        self.last_state_a = current_state_a
        
        # 펄스 카운터 업데이트
        if not self.direction:
            self.pulse_count += 1
        else:
            self.pulse_count -= 1
    
    def get_pulse_count(self):
        """현재 펄스 카운트 반환"""
        return self.pulse_count
    
    def get_direction(self):
        """현재 회전 방향 반환 (True: 정방향, False: 역방향)"""
        return self.direction
    
    def reset_count(self):
        """펄스 카운터 리셋"""
        self.pulse_count = 0
        print("펄스 카운터 리셋됨")
    
    def get_speed_rpm(self, pulses_per_revolution=20):
        """
        RPM 계산 (간단한 구현)
        
        Args:
            pulses_per_revolution (int): 회전당 펄스 수 (기본값: 20)
        
        Returns:
            float: RPM 값
        """
        # 실제로는 시간 간격을 측정하여 정확한 RPM 계산이 필요
        # 여기서는 간단한 예시로 펄스 카운트 기반 계산
        return abs(self.pulse_count) / pulses_per_revolution


class L298NDriver:
    """L298N 드라이버 클래스 - 모터 방향 및 속도 제어"""
    
    def __init__(self, in1_pin=4, in2_pin=5, enable_pin=6, pwm_freq=15000):
        """
        L298N 드라이버 초기화
        
        Args:
            in1_pin (int): IN1 핀 번호 (기본값: 4)
            in2_pin (int): IN2 핀 번호 (기본값: 5)
            enable_pin (int): Enable 핀 번호 (기본값: 6)
            pwm_freq (int): PWM 주파수 (기본값: 15000Hz)
        """
        self.in1_pin_num = in1_pin
        self.in2_pin_num = in2_pin
        self.enable_pin_num = enable_pin
        
        # 디지털 핀으로 설정 (IN1, IN2는 방향 제어용)
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        
        # Enable 핀만 PWM으로 설정 (속도 제어용)
        self.pwm_enable = PWM(Pin(enable_pin), freq=pwm_freq, duty=0)
        
        # 모터 정지
        self.stop()
        
        print("L298N 드라이버 초기화 완료")
        print(f"IN1: GPIO {in1_pin}")
        print(f"IN2: GPIO {in2_pin}")
        print(f"Enable: GPIO {enable_pin}")
        print(f"PWM 주파수: {pwm_freq}Hz")
    
    def forward(self, speed=50):
        """
        정방향 회전 (참고 예제 방식 - Enable 핀 PWM으로 속도 제어)
        
        Args:
            speed (int): 속도 (0-100, 기본값: 50)
        """
        speed = max(0, min(100, speed))  # 0-100 범위 제한
        
        # Enable 핀 PWM으로 속도 제어 (참고 예제 방식)
        self.pwm_enable.duty(self.duty_cycle(speed))
        
        # 방향 제어
        self.in1.value(1)
        self.in2.value(0)
        
        print(f"정방향 회전 - 속도: {speed}% (Enable PWM)")
    
    def backward(self, speed=50):
        """
        역방향 회전 (참고 예제 방식 - Enable 핀 PWM으로 속도 제어)
        
        Args:
            speed (int): 속도 (0-100, 기본값: 50)
        """
        speed = max(0, min(100, speed))  # 0-100 범위 제한
        
        # Enable 핀 PWM으로 속도 제어 (참고 예제 방식)
        self.pwm_enable.duty(self.duty_cycle(speed))
        
        # 방향 제어
        self.in1.value(0)
        self.in2.value(1)
        
        print(f"역방향 회전 - 속도: {speed}% (Enable PWM)")
    
    def stop(self):
        """모터 정지"""
        self.pwm_enable.duty(0)  # Enable 핀 PWM 비활성화
        self.in1.value(0)
        self.in2.value(0)
        print("모터 정지")
    
    def duty_cycle(self, speed):
        """
        속도를 duty cycle로 변환 (참고 예제 방식)
        
        Args:
            speed (int): 속도 (0-100)
        
        Returns:
            int: duty cycle (0-1023)
        """
        min_duty = 750  # 참고 예제 값
        max_duty = 1023
        
        if speed <= 0 or speed > 100:
            duty_cycle = 0
        else:
            duty_cycle = int(min_duty + (max_duty - min_duty) * ((speed - 1) / (100 - 1)))
        
        return duty_cycle
    
    def brake(self):
        """모터 브레이크 (단축)"""
        self.pwm_enable.duty(1023)  # Enable 핀 최대 출력
        self.in1.value(1)
        self.in2.value(1)
        print("모터 브레이크")
    


class MotorWithEncoder:
    """엔코더가 장착된 모터 통합 제어 클래스"""
    
    def __init__(self, encoder_pin_a=18, encoder_pin_b=17, 
                 motor_in1=4, motor_in2=5, motor_enable=6):
        """
        모터와 엔코더 통합 초기화
        
        Args:
            encoder_pin_a (int): 엔코더 Phase A 핀 (기본값: 18)
            encoder_pin_b (int): 엔코더 Phase B 핀 (기본값: 17)
            motor_in1 (int): 모터 드라이버 IN1 핀 (기본값: 4)
            motor_in2 (int): 모터 드라이버 IN2 핀 (기본값: 5)
            motor_enable (int): 모터 드라이버 Enable 핀 (기본값: 6)
        """
        self.encoder = Encoder(encoder_pin_a, encoder_pin_b)
        self.motor = L298NDriver(motor_in1, motor_in2, motor_enable)
        
        print("모터 엔코더 시스템 초기화 완료")
    
    def forward(self, speed=512):
        """정방향 회전"""
        self.motor.forward(speed)
    
    def backward(self, speed=512):
        """역방향 회전"""
        self.motor.backward(speed)
    
    def stop(self):
        """모터 정지"""
        self.motor.stop()
    
    def brake(self):
        """모터 브레이크"""
        self.motor.brake()
    
    def get_position(self):
        """현재 위치 (펄스 카운트) 반환"""
        return self.encoder.get_pulse_count()
    
    def get_direction(self):
        """현재 회전 방향 반환"""
        return self.encoder.get_direction()
    
    def reset_position(self):
        """위치 리셋"""
        self.encoder.reset_count()
    
    def get_speed_rpm(self, pulses_per_revolution=20):
        """RPM 계산"""
        return self.encoder.get_speed_rpm(pulses_per_revolution)
    
    def move_to_position(self, target_position, speed=512, tolerance=5):
        """
        특정 위치로 이동 (간단한 PID 제어)
        
        Args:
            target_position (int): 목표 위치
            speed (int): 속도 (0-1023)
            tolerance (int): 허용 오차
        
        Returns:
            bool: 목표 위치 도달 여부
        """
        current_position = self.get_position()
        error = target_position - current_position
        
        if abs(error) <= tolerance:
            self.stop()
            return True
        
        if error > 0:
            self.forward(speed)
        else:
            self.backward(speed)
        
        return False
    
    def print_status(self):
        """현재 상태 출력"""
        print("=== 모터 상태 ===")
        print(f"위치: {self.get_position()}")
        print(f"방향: {'정방향' if self.get_direction() else '역방향'}")
        print(f"RPM: {self.get_speed_rpm():.2f}")
        print("================")
