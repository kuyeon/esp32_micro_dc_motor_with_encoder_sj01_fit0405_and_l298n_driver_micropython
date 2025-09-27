"""
ESP32용 마이크로 DC 모터 엔코더 및 L298N 드라이버 라이브러리
SJ01 FIT0405 엔코더 모터와 L298N 드라이버를 제어합니다.

연결:
- 엔코더 Phase A: GPIO 18
- 엔코더 Phase B: GPIO 17 (인터럽트 핀)
- L298N IN1: GPIO 4
- L298N IN2: GPIO 5
- L298N Enable: GPIO 6
"""

import machine
import time
from machine import Pin, PWM


class PIDController:
    """PID 컨트롤러 클래스"""
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_min=-100, output_max=100):
        """
        PID 컨트롤러 초기화
        
        Args:
            kp (float): 비례 게인 (기본값: 1.0)
            ki (float): 적분 게인 (기본값: 0.0)
            kd (float): 미분 게인 (기본값: 0.0)
            output_min (float): 최소 출력값 (기본값: -100)
            output_max (float): 최대 출력값 (기본값: 100)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        
        # PID 상태 변수
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.ticks_ms()
        
        print(f"PID 컨트롤러 초기화 완료 - Kp:{kp}, Ki:{ki}, Kd:{kd}")
    
    def compute(self, setpoint, current_value):
        """
        PID 계산 수행
        
        Args:
            setpoint (float): 목표값
            current_value (float): 현재값
        
        Returns:
            float: PID 출력값 (제한된 범위 내)
        """
        # 현재 시간
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, self.last_time) / 1000.0  # 초 단위
        
        if dt <= 0:
            dt = 0.01  # 최소 시간 간격
        
        # 오차 계산
        error = setpoint - current_value
        
        # 비례 항
        proportional = self.kp * error
        
        # 적분 항
        self.integral += error * dt
        integral = self.ki * self.integral
        
        # 미분 항
        derivative = self.kd * (error - self.previous_error) / dt
        
        # PID 출력 계산
        output = proportional + integral + derivative
        
        # 출력 제한
        output = max(self.output_min, min(self.output_max, output))
        
        # 상태 업데이트
        self.previous_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        """PID 컨트롤러 리셋"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.ticks_ms()
        print("PID 컨트롤러 리셋됨")
    
    def set_parameters(self, kp=None, ki=None, kd=None):
        """
        PID 파라미터 설정
        
        Args:
            kp (float): 비례 게인
            ki (float): 적분 게인
            kd (float): 미분 게인
        """
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        
        print(f"PID 파라미터 업데이트 - Kp:{self.kp}, Ki:{self.ki}, Kd:{self.kd}")

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
        
        # RPM 계산을 위한 변수들
        self.last_pulse_count = 0
        self.last_time = time.ticks_ms()
        
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
        self.last_pulse_count = 0
        self.last_time = time.ticks_ms()
        print("펄스 카운터 리셋됨")
    
    def get_speed_rpm(self, pulses_per_revolution=1920):
        """
        RPM 계산 (간단한 구현)
        
        Args:
            pulses_per_revolution (int): 회전당 펄스 수 (기본값: 1920 - SJ01 FIT0405 스펙)
        
        Returns:
            float: RPM 값
        """
        # 실제로는 시간 간격을 측정하여 정확한 RPM 계산이 필요
        # 여기서는 간단한 예시로 펄스 카운트 기반 계산
        return abs(self.pulse_count) / pulses_per_revolution
    
    def get_current_rpm(self, pulses_per_revolution=1920):
        """
        현재 RPM 계산 (시간 간격 기반)
        
        Args:
            pulses_per_revolution (int): 회전당 펄스 수 (기본값: 1920 - SJ01 FIT0405 스펙)
        
        Returns:
            float: 현재 RPM 값
        """
        current_time = time.ticks_ms()
        time_diff = time.ticks_diff(current_time, self.last_time)
        
        if time_diff > 100:  # 100ms 이상 경과했을 때만 계산
            pulse_diff = self.pulse_count - self.last_pulse_count
            
            # RPM 계산: (펄스 차이 / 회전당 펄스 수) * (60000ms / 시간 차이ms)
            rpm = (abs(pulse_diff) / pulses_per_revolution) * (60000 / time_diff)
            
            # 다음 계산을 위한 값 업데이트
            self.last_pulse_count = self.pulse_count
            self.last_time = current_time
            
            return rpm
        
        return 0.0


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
        self.in1.value(0)
        self.in2.value(0)
        self.pwm_enable.duty(0)  # Enable 핀 PWM 비활성화
        
        print("모터 정지")
    
    def duty_cycle(self, speed):
        """
        속도를 duty cycle로 변환 (안전한 범위로 제한)
        
        Args:
            speed (int): 속도 (0-100)
        
        Returns:
            int: duty cycle (625-1023, 확장된 범위)
        """
        min_duty = 625  # 최소 duty
        max_duty = 1023  # 최대 duty
        
        if speed <= 0 or speed > 100:
            duty_cycle = 0
        else:
            duty_cycle = int(min_duty + (max_duty - min_duty) * ((speed - 1) / (100 - 1)))
        
        return duty_cycle
    
    def brake(self):
        """모터 브레이크 (최대 duty cycle 사용)"""
        self.pwm_enable.duty(1023)  # 최대 duty cycle 사용
        self.in1.value(1)
        self.in2.value(1)
        print("모터 브레이크")
    
    def safe_forward(self, speed=50):
        """
        안전한 정방향 회전 (duty cycle 제한)
        
        Args:
            speed (int): 속도 (0-100, 기본값: 50)
        """
        speed = max(0, min(100, speed))  # 0-100 범위 제한
        duty = self.duty_cycle(speed)
        
        # 안전한 duty cycle로 속도 제어
        self.pwm_enable.duty(duty)
        
        # 방향 제어
        self.in1.value(1)
        self.in2.value(0)
        
        print(f"안전한 정방향 회전 - 속도: {speed}% (duty: {duty})")
    
    def safe_backward(self, speed=50):
        """
        안전한 역방향 회전 (duty cycle 제한)
        
        Args:
            speed (int): 속도 (0-100, 기본값: 50)
        """
        speed = max(0, min(100, speed))  # 0-100 범위 제한
        duty = self.duty_cycle(speed)
        
        # 안전한 duty cycle로 속도 제어
        self.pwm_enable.duty(duty)
        
        # 방향 제어
        self.in1.value(0)
        self.in2.value(1)
        
        print(f"안전한 역방향 회전 - 속도: {speed}% (duty: {duty})")
    


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
        
        # PID 컨트롤러 초기화
        self.position_pid = PIDController(kp=0.5, ki=0.0, kd=0.1, output_min=-100, output_max=100)
        self.speed_pid = PIDController(kp=1.0, ki=0.1, kd=0.05, output_min=-100, output_max=100)
        
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
    
    def get_speed_rpm(self, pulses_per_revolution=1920):
        """RPM 계산 (기본)"""
        return self.encoder.get_speed_rpm(pulses_per_revolution)
    
    def get_current_rpm(self, pulses_per_revolution=1920):
        """현재 RPM 계산 (시간 간격 기반)"""
        return self.encoder.get_current_rpm(pulses_per_revolution)
    
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
    
    def move_to_position_pid(self, target_position, tolerance=5):
        """
        PID 제어를 사용한 정밀한 위치 제어
        
        Args:
            target_position (int): 목표 위치
            tolerance (int): 허용 오차
        
        Returns:
            bool: 목표 위치 도달 여부
        """
        current_position = self.get_position()
        
        if abs(target_position - current_position) <= tolerance:
            self.stop()
            return True
        
        # PID 계산
        pid_output = self.position_pid.compute(target_position, current_position)
        
        # PID 출력을 속도로 변환 (절댓값)
        speed = abs(pid_output)
        speed = max(10, min(100, speed))  # 최소 10%, 최대 100%
        
        # 방향에 따라 모터 제어
        if pid_output > 0:
            self.forward(speed)
        else:
            self.backward(speed)
        
        return False
    
    def control_speed_pid(self, target_rpm):
        """
        PID 제어를 사용한 속도 제어
        
        Args:
            target_rpm (float): 목표 RPM
        
        Returns:
            float: 현재 RPM
        """
        current_rpm = self.get_current_rpm()
        
        # PID 계산
        pid_output = self.speed_pid.compute(target_rpm, current_rpm)
        
        # PID 출력을 속도로 변환
        speed = abs(pid_output)
        speed = max(10, min(100, speed))  # 최소 10%, 최대 100%
        
        # 방향에 따라 모터 제어
        if pid_output > 0:
            self.forward(speed)
        else:
            self.backward(speed)
        
        return current_rpm
    
    def set_position_pid_parameters(self, kp=None, ki=None, kd=None):
        """위치 제어 PID 파라미터 설정"""
        self.position_pid.set_parameters(kp, ki, kd)
    
    def set_speed_pid_parameters(self, kp=None, ki=None, kd=None):
        """속도 제어 PID 파라미터 설정"""
        self.speed_pid.set_parameters(kp, ki, kd)
    
    def reset_pid_controllers(self):
        """모든 PID 컨트롤러 리셋"""
        self.position_pid.reset()
        self.speed_pid.reset()
    
    def print_status(self):
        """현재 상태 출력"""
        print("=== 모터 상태 ===")
        print(f"위치: {self.get_position()}")
        print(f"방향: {'정방향' if self.get_direction() else '역방향'}")
        print(f"RPM: {self.get_speed_rpm():.2f}")
        print("================")
