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

if __name__ == "__main__":
    # 기본 사용 예제 실행
    basic_example()
    
    # 다른 예제들을 실행하려면 아래 주석 해제
    # encoder_example()
    # motor_only_example()
    # position_control_example()
