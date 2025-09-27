"""
ESP32 마이크로 DC 모터 엔코더 라이브러리 사용 예제
"""

from motor_encoder import MotorWithEncoder, Encoder, L298NDriver
import time

def basic_example():
    """기본 사용 예제"""
    print("=== 기본 사용 예제 ===")
    
    # 모터 엔코더 객체 생성
    motor = MotorWithEncoder()
    
    print("정방향 회전 (5초)")
    motor.forward(50)  # 50% 속도로 정방향 회전
    time.sleep(5)
    
    print("역방향 회전 (5초)")
    motor.backward(50)  # 50% 속도로 역방향 회전
    time.sleep(5)
    
    print("정지")
    motor.stop()
    
    # 현재 상태 출력
    motor.print_status()

def encoder_example():
    """엔코더 사용 예제"""
    print("=== 엔코더 사용 예제 ===")
    
    # 엔코더만 사용
    encoder = Encoder(pin_a=18, pin_b=17)
    
    print("엔코더를 수동으로 회전시켜보세요...")
    
    for i in range(10):
        print(f"펄스: {encoder.get_pulse_count()}, 방향: {'정방향' if encoder.get_direction() else '역방향'}")
        time.sleep(1)
    
    encoder.reset_count()
    print("엔코더 카운터 리셋됨")

def motor_only_example():
    """모터만 사용 예제"""
    print("=== 모터만 사용 예제 ===")
    
    # L298N 드라이버만 사용
    motor = L298NDriver(in1_pin=4, in2_pin=5, enable_pin=6)
    time.sleep(1)
    print("정방향 회전 (3초)")
    motor.forward(70)  # 70% 속도
    time.sleep(3)
    
    print("역방향 회전 (3초)")
    motor.backward(70)  # 70% 속도
    time.sleep(3)
    
    print("정지")
    motor.stop()

def motor_debug_example():
    """모터 디버깅 예제 - 핀 상태 확인"""
    print("=== 모터 디버깅 예제 ===")
    
    motor = L298NDriver(in1_pin=4, in2_pin=5, enable_pin=6)
    
    print("\n1. 정방향 회전 테스트 (단계별)")
    print("Enable PWM 활성화...")
    motor.pwm_enable.duty(motor.duty_cycle(70))
    print(f"Enable PWM duty: {motor.pwm_enable.duty()}")
    
    print("IN1 HIGH, IN2 LOW 설정...")
    motor.in1.value(1)
    motor.in2.value(0)
    print(f"IN1: {motor.in1.value()}, IN2: {motor.in2.value()}")
    
    time.sleep(3)
    
    print("\n2. 정지")
    motor.stop()
    print(f"Enable PWM duty: {motor.pwm_enable.duty()}")
    print(f"IN1: {motor.in1.value()}, IN2: {motor.in2.value()}")
    
    time.sleep(1)
    
    print("\n3. 역방향 회전 테스트 (단계별)")
    print("Enable PWM 활성화...")
    motor.pwm_enable.duty(motor.duty_cycle(70))
    print(f"Enable PWM duty: {motor.pwm_enable.duty()}")
    
    print("IN1 LOW, IN2 HIGH 설정...")
    motor.in1.value(0)
    motor.in2.value(1)
    print(f"IN1: {motor.in1.value()}, IN2: {motor.in2.value()}")
    
    time.sleep(3)
    
    print("\n4. 정지")
    motor.stop()
    print("디버깅 완료")

def motor_pin_test():
    """핀별 개별 테스트"""
    print("=== 핀별 개별 테스트 ===")
    
    from machine import Pin, PWM
    
    # 각 핀을 직접 제어
    in1 = Pin(4, Pin.OUT)
    in2 = Pin(5, Pin.OUT)
    enable = PWM(Pin(6), freq=15000, duty=0)
    
    print("1. Enable 핀만 테스트")
    enable.duty(750)  # 70% 속도에 해당하는 duty
    time.sleep(2)
    enable.duty(0)
    
    print("2. Enable + IN1 테스트")
    enable.duty(750)
    in1.value(1)
    in2.value(0)
    time.sleep(3)
    
    print("3. 모든 핀 LOW")
    enable.duty(0)
    in1.value(0)
    in2.value(0)
    time.sleep(3)
    print("4. Enable + IN2 테스트")
    enable.duty(750)
    in1.value(0)
    in2.value(1)
    time.sleep(3)
    
    print("5. 정지")
    enable.duty(0)
    in1.value(0)
    in2.value(0)
    
    print("핀별 테스트 완료")

def safe_motor_example():
    """안전한 모터 제어 예제 (duty cycle 제한)"""
    print("=== 안전한 모터 제어 예제 ===")
    
    motor = L298NDriver(in1_pin=4, in2_pin=5, enable_pin=6)
    
    print("안전한 정방향 회전 (3초)")
    motor.safe_forward(70)  # 70% 속도 (duty cycle 제한됨)
    time.sleep(3)
    
    print("안전한 역방향 회전 (3초)")
    motor.safe_backward(70)  # 70% 속도 (duty cycle 제한됨)
    time.sleep(3)
    
    print("정지")
    motor.stop()
    
    print("안전한 모터 제어 완료")

def duty_cycle_test():
    """Duty cycle 범위 테스트"""
    print("=== Duty Cycle 범위 테스트 ===")
    
    motor = L298NDriver(in1_pin=4, in2_pin=5, enable_pin=6)
    
    # 다양한 속도로 duty cycle 확인
    speeds = [10, 25, 50, 75, 100]
    
    for speed in speeds:
        duty = motor.duty_cycle(speed)
        print(f"속도 {speed}% → Duty Cycle: {duty}")
        
        # 각 속도로 1초간 테스트
        print(f"  정방향 테스트...")
        motor.safe_forward(speed)
        time.sleep(1)
        
        print(f"  역방향 테스트...")
        motor.safe_backward(speed)
        time.sleep(1)
        
        motor.stop()
        time.sleep(0.5)
    
    print("Duty cycle 테스트 완료")

def find_minimum_duty_test():
    """최소 duty cycle 찾기 테스트"""
    print("=== 최소 Duty Cycle 찾기 테스트 ===")
    
    motor = L298NDriver(in1_pin=4, in2_pin=5, enable_pin=6)
    
    # 정방향 최소 duty 찾기
    print("\n1. 정방향 최소 duty cycle 찾기")
    min_duty_forward = None
    
    for duty in range(0, 1023, 100):  # 0부터 800까지 10씩 증가
        print(f"정방향 duty {duty} 테스트...")
        
        motor.pwm_enable.duty(duty)
        motor.in1.value(1)
        motor.in2.value(0)
        
        time.sleep(1)  # 1초간 관찰
        
        # 여기서 사용자가 모터가 움직이는지 확인
        # 실제로는 자동으로 감지할 수 있지만, 수동 확인을 위해
        motor.stop()
        time.sleep(0.5)
        
        if duty >= 1000:  # 100 이상에서 테스트 시작
            break
    
    print(f"정방향 최소 duty cycle: {min_duty_forward}")
    
    # 역방향 최소 duty 찾기
    print("\n2. 역방향 최소 duty cycle 찾기")
    min_duty_backward = None
    
    for duty in range(0, 1023, 100):  # 0부터 800까지 10씩 증가
        print(f"역방향 duty {duty} 테스트...")
        
        motor.pwm_enable.duty(duty)
        motor.in1.value(0)
        motor.in2.value(1)
        
        time.sleep(1)  # 1초간 관찰
        
        motor.stop()
        time.sleep(0.5)
        
        if duty >= 1000:  # 100 이상에서 테스트 시작
            break
    
    print(f"역방향 최소 duty cycle: {min_duty_backward}")
    
    # 최소 duty cycle 적용
    if min_duty_forward is not None and min_duty_backward is not None:
        min_duty = min(min_duty_forward, min_duty_backward)
        print(f"\n권장 최소 duty cycle: {min_duty}")
        
        # 새로운 duty_cycle 함수 테스트
        print("\n3. 새로운 duty cycle 함수 테스트")
        test_speeds = [10, 25, 50, 75, 100]
        
        for speed in test_speeds:
            # 임시로 최소 duty 변경
            original_min = motor.duty_cycle.__code__.co_consts[0] if hasattr(motor.duty_cycle.__code__, 'co_consts') else 750
            duty = int(min_duty + (800 - min_duty) * ((speed - 1) / (100 - 1)))
            
            print(f"속도 {speed}% → Duty: {duty}")
    
    motor.stop()
    print("최소 duty cycle 찾기 완료")

def detailed_duty_test():
    """상세한 duty cycle 테스트"""
    print("=== 상세 Duty Cycle 테스트 ===")
    
    motor = L298NDriver(in1_pin=4, in2_pin=5, enable_pin=6)
    
    # 정밀한 duty cycle 테스트
    duty_range = [0, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700, 750, 800]
    
    print("\n정방향 duty cycle 테스트:")
    for duty in duty_range:
        print(f"Duty {duty}: ", end="")
        
        motor.pwm_enable.duty(duty)
        motor.in1.value(1)
        motor.in2.value(0)
        
        time.sleep(0.5)  # 0.5초간 테스트
        motor.stop()
        
        # 여기서 사용자가 모터 동작 여부를 확인
        print("동작 확인")
        time.sleep(0.2)
    
    print("\n역방향 duty cycle 테스트:")
    for duty in duty_range:
        print(f"Duty {duty}: ", end="")
        
        motor.pwm_enable.duty(duty)
        motor.in1.value(0)
        motor.in2.value(1)
        
        time.sleep(0.5)  # 0.5초간 테스트
        motor.stop()
        
        print("동작 확인")
        time.sleep(0.2)
    
    motor.stop()
    print("상세 duty cycle 테스트 완료")

def auto_minimum_duty_test():
    """자동 최소 duty cycle 찾기 (엔코더 기반)"""
    print("=== 자동 최소 Duty Cycle 찾기 ===")
    
    # 엔코더가 있는 모터 사용
    motor_encoder = MotorWithEncoder()
    
    print("\n정방향 최소 duty 찾기:")
    min_duty_forward = None
    
    # 정방향 테스트 (300부터 시작)
    for duty in range(300, 800, 25):  # 300부터 시작, 25씩 증가
        print(f"정방향 duty {duty} 테스트...")
        
        # 위치 리셋
        motor_encoder.reset_position()
        initial_pos = motor_encoder.get_position()
        
        # 모터 시작
        motor_encoder.motor.pwm_enable.duty(duty)
        motor_encoder.motor.in1.value(1)
        motor_encoder.motor.in2.value(0)
        
        time.sleep(3)  # 3초간 실행 (더 긴 시간)
        
        # 위치 확인
        final_pos = motor_encoder.get_position()
        movement = abs(final_pos - initial_pos)
        
        motor_encoder.stop()
        time.sleep(1)  # 충분한 정지 시간
        
        print(f"  위치 변화: {movement} 펄스")
        
        if movement > 10 and min_duty_forward is None:  # 10펄스 이상 움직이면
            min_duty_forward = duty
            print(f"  ✓ 정방향 최소 duty 발견: {duty}")
            break  # 최소 duty 찾으면 중단
        
        # 800까지 모든 duty 테스트
    
    print("\n역방향 최소 duty 찾기:")
    min_duty_backward = None
    
    # 역방향 테스트 (300부터 시작)
    for duty in range(300, 800, 25):  # 300부터 시작, 25씩 증가
        print(f"역방향 duty {duty} 테스트...")
        
        # 위치 리셋
        motor_encoder.reset_position()
        initial_pos = motor_encoder.get_position()
        
        # 모터 시작
        motor_encoder.motor.pwm_enable.duty(duty)
        motor_encoder.motor.in1.value(0)
        motor_encoder.motor.in2.value(1)
        
        time.sleep(3)  # 3초간 실행
        
        # 위치 확인
        final_pos = motor_encoder.get_position()
        movement = abs(final_pos - initial_pos)
        
        motor_encoder.stop()
        time.sleep(1)  # 충분한 정지 시간
        
        print(f"  위치 변화: {movement} 펄스")
        
        if movement > 10 and min_duty_backward is None:  # 10펄스 이상 움직이면
            min_duty_backward = duty
            print(f"  ✓ 역방향 최소 duty 발견: {duty}")
            break  # 최소 duty 찾으면 중단
        
        # 800까지 모든 duty 테스트
    
    # 결과 출력
    print(f"\n=== 결과 ===")
    print(f"정방향 최소 duty: {min_duty_forward}")
    print(f"역방향 최소 duty: {min_duty_backward}")
    
    if min_duty_forward is not None and min_duty_backward is not None:
        recommended_min = max(min_duty_forward, min_duty_backward)
        print(f"권장 최소 duty: {recommended_min}")
        
        # 새로운 duty_cycle 함수 생성 제안
        print(f"\n권장 duty_cycle 함수 설정:")
        print(f"min_duty = {recommended_min}")
        print(f"max_duty = 800")
        
        # 실제 duty_cycle 함수 테스트
        print(f"\n새로운 duty_cycle 함수 테스트:")
        test_speeds = [10, 25, 50, 75, 100]
        for speed in test_speeds:
            new_duty = int(recommended_min + (800 - recommended_min) * ((speed - 1) / (100 - 1)))
            print(f"속도 {speed}% → Duty: {new_duty}")
    
    elif min_duty_forward is not None:
        print(f"정방향만 감지됨. 권장 최소 duty: {min_duty_forward}")
    elif min_duty_backward is not None:
        print(f"역방향만 감지됨. 권장 최소 duty: {min_duty_backward}")
    else:
        print("모터가 움직이지 않습니다. 하드웨어 연결을 확인하세요.")
    
    motor_encoder.stop()
    print("자동 최소 duty cycle 찾기 완료")

def precise_minimum_duty_test():
    """정밀한 최소 duty cycle 찾기 (엔코더 기반)"""
    print("=== 정밀한 최소 Duty Cycle 찾기 ===")
    
    motor_encoder = MotorWithEncoder()
    
    # 정방향 정밀 테스트
    print("\n정방향 정밀 테스트:")
    min_duty_forward = None
    
    # 큰 간격으로 대략적인 범위 찾기
    for duty in range(50, 300, 50):
        print(f"정방향 duty {duty} 대략 테스트...")
        
        motor_encoder.reset_position()
        initial_pos = motor_encoder.get_position()
        
        motor_encoder.motor.pwm_enable.duty(duty)
        motor_encoder.motor.in1.value(1)
        motor_encoder.motor.in2.value(0)
        
        time.sleep(2)
        
        final_pos = motor_encoder.get_position()
        movement = abs(final_pos - initial_pos)
        
        motor_encoder.stop()
        time.sleep(0.5)
        
        print(f"  위치 변화: {movement}")
        
        if movement > 5:
            # 이 범위에서 정밀 테스트
            print(f"  → duty {duty-50}~{duty} 범위에서 정밀 테스트 시작")
            for precise_duty in range(max(50, duty-50), duty+1, 10):
                print(f"    정밀 테스트 duty {precise_duty}...")
                
                motor_encoder.reset_position()
                initial_pos = motor_encoder.get_position()
                
                motor_encoder.motor.pwm_enable.duty(precise_duty)
                motor_encoder.motor.in1.value(1)
                motor_encoder.motor.in2.value(0)
                
                time.sleep(2)
                
                final_pos = motor_encoder.get_position()
                movement = abs(final_pos - initial_pos)
                
                motor_encoder.stop()
                time.sleep(0.5)
                
                print(f"      위치 변화: {movement}")
                
                if movement > 5 and min_duty_forward is None:
                    min_duty_forward = precise_duty
                    print(f"      ✓ 정방향 최소 duty 발견: {precise_duty}")
                    break
            
            if min_duty_forward is not None:
                break
    
    # 역방향 정밀 테스트
    print("\n역방향 정밀 테스트:")
    min_duty_backward = None
    
    for duty in range(50, 300, 50):
        print(f"역방향 duty {duty} 대략 테스트...")
        
        motor_encoder.reset_position()
        initial_pos = motor_encoder.get_position()
        
        motor_encoder.motor.pwm_enable.duty(duty)
        motor_encoder.motor.in1.value(0)
        motor_encoder.motor.in2.value(1)
        
        time.sleep(2)
        
        final_pos = motor_encoder.get_position()
        movement = abs(final_pos - initial_pos)
        
        motor_encoder.stop()
        time.sleep(0.5)
        
        print(f"  위치 변화: {movement}")
        
        if movement > 5:
            # 이 범위에서 정밀 테스트
            print(f"  → duty {duty-50}~{duty} 범위에서 정밀 테스트 시작")
            for precise_duty in range(max(50, duty-50), duty+1, 10):
                print(f"    정밀 테스트 duty {precise_duty}...")
                
                motor_encoder.reset_position()
                initial_pos = motor_encoder.get_position()
                
                motor_encoder.motor.pwm_enable.duty(precise_duty)
                motor_encoder.motor.in1.value(0)
                motor_encoder.motor.in2.value(1)
                
                time.sleep(2)
                
                final_pos = motor_encoder.get_position()
                movement = abs(final_pos - initial_pos)
                
                motor_encoder.stop()
                time.sleep(0.5)
                
                print(f"      위치 변화: {movement}")
                
                if movement > 5 and min_duty_backward is None:
                    min_duty_backward = precise_duty
                    print(f"      ✓ 역방향 최소 duty 발견: {precise_duty}")
                    break
            
            if min_duty_backward is not None:
                break
    
    # 결과 출력
    print(f"\n=== 정밀 테스트 결과 ===")
    print(f"정방향 최소 duty: {min_duty_forward}")
    print(f"역방향 최소 duty: {min_duty_backward}")
    
    if min_duty_forward is not None and min_duty_backward is not None:
        recommended_min = max(min_duty_forward, min_duty_backward)
        print(f"권장 최소 duty: {recommended_min}")
        
        # 안전 마진 추가 (10% 여유)
        safe_min_duty = int(recommended_min * 1.1)
        print(f"안전 마진 포함 최소 duty: {safe_min_duty}")
        
    elif min_duty_forward is not None:
        print(f"정방향만 감지됨. 권장 최소 duty: {min_duty_forward}")
        safe_min_duty = int(min_duty_forward * 1.1)
        print(f"안전 마진 포함 최소 duty: {safe_min_duty}")
    elif min_duty_backward is not None:
        print(f"역방향만 감지됨. 권장 최소 duty: {min_duty_backward}")
        safe_min_duty = int(min_duty_backward * 1.1)
        print(f"안전 마진 포함 최소 duty: {safe_min_duty}")
    else:
        print("모터가 움직이지 않습니다. 하드웨어 연결을 확인하세요.")
        safe_min_duty = None
    
    motor_encoder.stop()
    print("정밀한 최소 duty cycle 찾기 완료")
    
    return safe_min_duty

def hardware_check_test():
    """하드웨어 연결 확인 테스트"""
    print("=== 하드웨어 연결 확인 테스트 ===")
    
    motor = L298NDriver(in1_pin=4, in2_pin=5, enable_pin=6)
    
    print("\n1. 핀 상태 확인:")
    print(f"IN1 핀 (GPIO 4): {motor.in1.value()}")
    print(f"IN2 핀 (GPIO 5): {motor.in2.value()}")
    print(f"Enable 핀 (GPIO 6): {motor.pwm_enable.duty()}")
    
    print("\n2. 고 duty cycle 테스트 (750, 800, 850, 900, 950, 1000):")
    test_duties = [750, 800, 850, 900, 950, 1000]
    
    for duty in test_duties:
        print(f"\nDuty {duty} 테스트:")
        
        # 정방향 테스트
        print("  정방향 2초...")
        motor.pwm_enable.duty(duty)
        motor.in1.value(1)
        motor.in2.value(0)
        time.sleep(2)
        
        # 역방향 테스트
        print("  역방향 2초...")
        motor.in1.value(0)
        motor.in2.value(1)
        time.sleep(2)
        
        # 정지
        motor.stop()
        time.sleep(0.5)
    
    print("\n3. Enable 핀 점프 연결 확인:")
    print("L298N 모듈에서 Enable 핀의 점퍼를 제거했는지 확인하세요.")
    print("점퍼가 연결되어 있으면 PWM 제어가 작동하지 않습니다.")
    
    print("\n4. 전원 공급 확인:")
    print("- L298N의 VCC와 GND가 올바르게 연결되었는지 확인")
    print("- 모터 전원(VS)과 논리 전원(VCC)이 분리되어 있는지 확인")
    print("- 전원 공급 장치의 전류 용량이 충분한지 확인")
    
    motor.stop()
    print("하드웨어 연결 확인 완료")

def extended_duty_test():
    """확장된 duty cycle 테스트 (300-800)"""
    print("=== 확장된 Duty Cycle 테스트 (300-800) ===")
    
    motor = L298NDriver(in1_pin=4, in2_pin=5, enable_pin=6)
    
    # 300부터 800까지 50씩 증가
    duty_range = range(300, 850, 50)
    
    print("\n정방향 테스트:")
    for duty in duty_range:
        print(f"Duty {duty}: ", end="")
        
        motor.pwm_enable.duty(duty)
        motor.in1.value(1)
        motor.in2.value(0)
        
        time.sleep(1)  # 1초간 테스트
        motor.stop()
        
        print("동작 확인")
        time.sleep(0.3)
    
    print("\n역방향 테스트:")
    for duty in duty_range:
        print(f"Duty {duty}: ", end="")
        
        motor.pwm_enable.duty(duty)
        motor.in1.value(0)
        motor.in2.value(1)
        
        time.sleep(1)  # 1초간 테스트
        motor.stop()
        
        print("동작 확인")
        time.sleep(0.3)
    
    motor.stop()
    print("확장된 duty cycle 테스트 완료")
# 전원 측정 예제 추가
def power_supply_test():
    """전원 공급 테스트"""
    print("=== 전원 공급 테스트 ===")
    
    motor = L298NDriver(in1_pin=4, in2_pin=5, enable_pin=6)
    
    # 점진적 duty cycle 증가
    duty_cycles = [750, 800, 850, 900, 950, 1000, 1023]
    
    for duty in duty_cycles:
        print(f"\nDuty Cycle: {duty} 테스트")
        
        # 정방향 테스트
        motor.pwm_enable.duty(duty)
        motor.in1.value(1)
        motor.in2.value(0)
        
        print("정방향 1초...")
        time.sleep(1)
        
        # 역방향 테스트
        motor.in1.value(0)
        motor.in2.value(1)
        
        print("역방향 1초...")
        time.sleep(1)
        
        # 정지
        motor.stop()
        time.sleep(0.5)
    
    print("전원 공급 테스트 완료")

def position_control_example():
    """위치 제어 예제"""
    print("=== 위치 제어 예제 ===")
    
    motor_encoder = MotorWithEncoder()
    motor_encoder.reset_position()
    
    # 목표 위치로 이동
    target_position = 100
    print(f"목표 위치 {target_position}로 이동")
    
    while not motor_encoder.move_to_position(target_position, speed=40, tolerance=5):
        current_pos = motor_encoder.get_position()
        print(f"현재 위치: {current_pos}, 목표: {target_position}")
        time.sleep(0.1)
    
    motor_encoder.print_status()

def rpm_monitoring_example():
    """RPM 모니터링 예제"""
    print("=== RPM 모니터링 예제 ===")
    
    motor_encoder = MotorWithEncoder()
    motor_encoder.reset_position()
    
    # 다양한 속도로 RPM 측정
    speeds = [30, 50, 70, 90, 100]
    
    for speed in speeds:
        print(f"\n속도 {speed}%로 5초간 회전하면서 RPM 측정")
        motor_encoder.forward(speed)
        
        # 5초간 RPM 측정
        for i in range(10):
            # 위치 리셋하여 정확한 RPM 계산
            if i == 0:
                motor_encoder.reset_position()
            
            time.sleep(0.5)  # 0.5초 대기
            
            # 현재 상태 출력
            position = motor_encoder.get_position()
            rpm = motor_encoder.get_current_rpm()  # 새로운 시간 기반 RPM 계산 사용
            direction = "정방향" if motor_encoder.get_direction() else "역방향"
            
            print(f"시간: {i*0.5:.1f}s, 위치: {position}, RPM: {rpm:.1f}, 방향: {direction}")
        
        motor_encoder.stop()
        time.sleep(1)
    
    print("\nRPM 모니터링 완료")

def real_time_rpm_example():
    """실시간 RPM 측정 예제"""
    print("=== 실시간 RPM 측정 예제 ===")
    
    motor_encoder = MotorWithEncoder()
    motor_encoder.reset_position()
    
    print("정방향으로 10초간 회전하면서 실시간 RPM 측정")
    motor_encoder.forward(60)  # 60% 속도
    
    start_time = time.time()
    last_position = 0
    
    while time.time() - start_time < 10:
        current_time = time.time() - start_time
        current_position = motor_encoder.get_position()
        
        # RPM 계산 (간단한 방법)
        if current_time > 0.5:  # 0.5초 후부터 계산
            position_diff = current_position - last_position
            time_diff = 0.1  # 0.1초 간격
            rpm = (abs(position_diff) / 20) * (60 / time_diff)  # 20펄스/회전 가정
            
            print(f"시간: {current_time:.1f}s, 위치: {current_position}, RPM: {rpm:.1f}")
        
        last_position = current_position
        time.sleep(0.1)
    
    motor_encoder.stop()
    print("실시간 RPM 측정 완료")

def speed_ramp_example():
    """속도 램프 예제 - 점진적 속도 증가/감소"""
    print("=== 속도 램프 예제 ===")
    
    motor_encoder = MotorWithEncoder()
    motor_encoder.reset_position()
    
    print("정방향으로 속도 점진적 증가 (0% → 100%)")
    
    # 속도 점진적 증가
    for speed in range(0, 101, 10):
        print(f"속도: {speed}%")
        motor_encoder.forward(speed)
        
        # 1초간 측정
        time.sleep(1)
        
        position = motor_encoder.get_position()
        rpm = motor_encoder.get_current_rpm()  # 새로운 시간 기반 RPM 계산 사용
        print(f"  위치: {position}, RPM: {rpm:.1f}")
    
    print("속도 점진적 감소 (100% → 0%)")
    
    # 속도 점진적 감소
    for speed in range(100, -1, -10):
        print(f"속도: {speed}%")
        motor_encoder.forward(speed)
        
        # 1초간 측정
        time.sleep(1)
        
        position = motor_encoder.get_position()
        rpm = motor_encoder.get_current_rpm()  # 새로운 시간 기반 RPM 계산 사용
        print(f"  위치: {position}, RPM: {rpm:.1f}")
    
    motor_encoder.stop()
    print("속도 램프 테스트 완료")

def pid_position_control_example():
    """PID 위치 제어 예제"""
    print("=== PID 위치 제어 예제 ===")
    
    motor_encoder = MotorWithEncoder()
    motor_encoder.reset_position()
    
    # PID 파라미터 설정 (필요시 조정)
    motor_encoder.set_position_pid_parameters(kp=0.8, ki=0.0, kd=0.2)
    
    # 여러 목표 위치로 이동
    target_positions = [100, -50, 200, 0, 150, -100]
    
    for target in target_positions:
        print(f"\n목표 위치: {target}로 PID 제어 이동")
        
        start_time = time.time()
        while not motor_encoder.move_to_position_pid(target, tolerance=3):
            current_pos = motor_encoder.get_position()
            elapsed = time.time() - start_time
            
            print(f"시간: {elapsed:.1f}s, 현재 위치: {current_pos}, 목표: {target}")
            time.sleep(0.1)
            
            # 타임아웃 체크 (10초)
            if elapsed > 10:
                print("타임아웃 발생")
                break
        
        motor_encoder.print_status()
        time.sleep(2)
    
    motor_encoder.stop()
    print("PID 위치 제어 완료")

def pid_speed_control_example():
    """PID 속도 제어 예제"""
    print("=== PID 속도 제어 예제 ===")
    
    motor_encoder = MotorWithEncoder()
    motor_encoder.reset_position()
    
    # PID 파라미터 설정 (필요시 조정)
    motor_encoder.set_speed_pid_parameters(kp=2.0, ki=0.1, kd=0.1)
    
    # 다양한 목표 RPM으로 테스트
    target_rpms = [50, 100, 150, 200, 100, 0]
    
    for target_rpm in target_rpms:
        if target_rpm == 0:
            print(f"\n모터 정지")
            motor_encoder.stop()
            time.sleep(2)
            continue
        
        print(f"\n목표 RPM: {target_rpm}으로 PID 제어")
        
        # 5초간 목표 RPM 유지
        for i in range(50):  # 0.1초 간격으로 5초
            current_rpm = motor_encoder.control_speed_pid(target_rpm)
            
            if i % 10 == 0:  # 1초마다 출력
                print(f"시간: {i*0.1:.1f}s, 현재 RPM: {current_rpm:.1f}, 목표: {target_rpm}")
            
            time.sleep(0.1)
    
    motor_encoder.stop()
    print("PID 속도 제어 완료")

def pid_tuning_example():
    """PID 튜닝 예제"""
    print("=== PID 튜닝 예제 ===")
    
    motor_encoder = MotorWithEncoder()
    motor_encoder.reset_position()
    
    # 다양한 PID 파라미터로 테스트
    pid_params = [
        {"kp": 0.5, "ki": 0.0, "kd": 0.0, "name": "P 제어만"},
        {"kp": 0.5, "ki": 0.1, "kd": 0.0, "name": "PI 제어"},
        {"kp": 0.5, "ki": 0.1, "kd": 0.1, "name": "PID 제어"},
        {"kp": 1.0, "ki": 0.1, "kd": 0.2, "name": "강화된 PID"}
    ]
    
    for params in pid_params:
        print(f"\n=== {params['name']} 테스트 ===")
        
        # PID 파라미터 설정
        motor_encoder.set_position_pid_parameters(
            kp=params["kp"], 
            ki=params["ki"], 
            kd=params["kd"]
        )
        
        # 목표 위치로 이동 테스트
        target = 100
        motor_encoder.reset_position()
        
        start_time = time.time()
        while not motor_encoder.move_to_position_pid(target, tolerance=5):
            current_pos = motor_encoder.get_position()
            elapsed = time.time() - start_time
            
            if elapsed > 5:  # 5초 타임아웃
                print("타임아웃")
                break
            
            time.sleep(0.1)
        
        final_pos = motor_encoder.get_position()
        error = abs(target - final_pos)
        print(f"최종 위치: {final_pos}, 오차: {error}")
        
        time.sleep(1)
    
    motor_encoder.stop()
    print("PID 튜닝 테스트 완료")

if __name__ == "__main__":
    # 기본 사용 예제 실행
    basic_example()
    
    # 다른 예제들을 실행하려면 아래 주석 해제
    # encoder_example()
    # motor_only_example()
    # motor_debug_example()  # 모터 디버깅
    # motor_pin_test()  # 핀별 테스트
    # safe_motor_example()  # 안전한 모터 제어
    # duty_cycle_test()  # Duty cycle 범위 테스트
    # find_minimum_duty_test()  # 수동 최소 duty 찾기
    # detailed_duty_test()  # 상세 duty 테스트
    # auto_minimum_duty_test()  # 자동 최소 duty 찾기 (엔코더 기반)
    # precise_minimum_duty_test()  # 정밀한 최소 duty 찾기 (엔코더 기반)
    # hardware_check_test()  # 하드웨어 연결 확인
    # extended_duty_test()  # 확장된 duty 테스트 (300-800)
    # position_control_example()
    # rpm_monitoring_example()
    # real_time_rpm_example()
    # speed_ramp_example()
    # pid_position_control_example()
    # pid_speed_control_example()
    # pid_tuning_example()
