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
    
    print("정방향 회전 (3초)")
    motor.forward(70)  # 70% 속도
    time.sleep(3)
    
    print("역방향 회전 (3초)")
    motor.backward(70)  # 70% 속도
    time.sleep(3)
    
    print("정지")
    motor.stop()

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
    speeds = [30, 50, 70, 90]
    
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

if __name__ == "__main__":
    # 기본 사용 예제 실행
    basic_example()
    
    # 다른 예제들을 실행하려면 아래 주석 해제
    # encoder_example()
    # motor_only_example()
    # position_control_example()
    # rpm_monitoring_example()
    # real_time_rpm_example()
    # speed_ramp_example()
