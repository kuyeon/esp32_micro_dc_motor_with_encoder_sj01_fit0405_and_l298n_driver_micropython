# ESP32 마이크로 DC 모터 엔코더 라이브러리

ESP32와 L298N 드라이버를 사용한 마이크로 DC 모터(SJ01 FIT0405) 엔코더 제어 라이브러리입니다.

## 하드웨어 연결

### 엔코더 연결
- **Phase A**: GPIO 18
- **Phase B**: GPIO 17 (인터럽트 핀)

### L298N 드라이버 연결
- **IN1**: GPIO 4 (디지털 핀)
- **IN2**: GPIO 5 (디지털 핀)
- **Enable**: GPIO 6 (PWM 핀)

## 라이브러리 구성

### 1. Encoder 클래스
엔코더 펄스 카운팅 및 방향 감지 기능

```python
from motor_encoder import Encoder

# 엔코더 초기화
encoder = Encoder(pin_a=18, pin_b=17)

# 펄스 카운트 읽기
count = encoder.get_pulse_count()

# 회전 방향 확인
direction = encoder.get_direction()  # True: 정방향, False: 역방향

# 카운터 리셋
encoder.reset_count()
```

### 2. L298NDriver 클래스
L298N 드라이버를 통한 모터 제어

```python
from motor_encoder import L298NDriver

# 드라이버 초기화
motor = L298NDriver(in1_pin=4, in2_pin=5, enable_pin=6)

# 정방향 회전 (속도: 0-100%)
motor.forward(50)

# 역방향 회전
motor.backward(50)

# 정지
motor.stop()

# 브레이크
motor.brake()
```

### 3. MotorWithEncoder 클래스
엔코더와 모터를 통합 제어하는 클래스

```python
from motor_encoder import MotorWithEncoder

# 통합 모터 제어 객체 생성
motor_encoder = MotorWithEncoder()

# 정방향 회전
motor_encoder.forward(500)

# 현재 위치 확인
position = motor_encoder.get_position()

# 상태 출력
motor_encoder.print_status()

# 특정 위치로 이동
motor_encoder.move_to_position(target_position=100, speed=400)

# 정지
motor_encoder.stop()
```

## 사용 예제

### 기본 사용법

```python
from motor_encoder import MotorWithEncoder
import time

# 모터 엔코더 객체 생성
motor = MotorWithEncoder()

# 정방향으로 5초간 회전 (50% 속도)
motor.forward(50)
time.sleep(5)

# 상태 확인
motor.print_status()

# 정지
motor.stop()
```

### 위치 제어 예제

```python
from motor_encoder import MotorWithEncoder
import time

motor = MotorWithEncoder()

# 목표 위치로 이동
target_position = 100
motor.move_to_position(target_position, speed=50, tolerance=5)

# 현재 위치 확인
current_pos = motor.get_position()
print(f"현재 위치: {current_pos}")
```

### 속도 제어 예제

```python
from motor_encoder import MotorWithEncoder
import time

motor = MotorWithEncoder()

# 다양한 속도로 회전 테스트
speeds = [20, 40, 60, 80]

for speed in speeds:
    print(f"속도 {speed}%로 회전")
    motor.forward(speed)
    time.sleep(2)
    motor.stop()
    time.sleep(1)
```

### RPM 모니터링 예제

```python
from motor_encoder import MotorWithEncoder
import time

motor = MotorWithEncoder()
motor.reset_position()

# 다양한 속도로 RPM 측정
speeds = [30, 50, 70, 90]

for speed in speeds:
    print(f"속도 {speed}%로 5초간 회전하면서 RPM 측정")
    motor.forward(speed)
    
    # 5초간 RPM 측정
    for i in range(10):
        time.sleep(0.5)
        position = motor.get_position()
        rpm = motor.get_speed_rpm()
        direction = "정방향" if motor.get_direction() else "역방향"
        
        print(f"시간: {i*0.5:.1f}s, 위치: {position}, RPM: {rpm:.1f}, 방향: {direction}")
    
    motor.stop()
    time.sleep(1)
```

### 실시간 RPM 측정 예제

```python
from motor_encoder import MotorWithEncoder
import time

motor = MotorWithEncoder()
motor.reset_position()

print("정방향으로 10초간 회전하면서 실시간 RPM 측정")
motor.forward(60)  # 60% 속도

start_time = time.time()
last_position = 0

while time.time() - start_time < 10:
    current_time = time.time() - start_time
    current_position = motor.get_position()
    
    # RPM 계산
    if current_time > 0.5:
        position_diff = current_position - last_position
        time_diff = 0.1
        rpm = (abs(position_diff) / 20) * (60 / time_diff)  # 20펄스/회전 가정
        
        print(f"시간: {current_time:.1f}s, 위치: {current_position}, RPM: {rpm:.1f}")
    
    last_position = current_position
    time.sleep(0.1)

motor.stop()
```

## 파일 설명

- **motor_encoder.py**: 메인 라이브러리 파일
- **example_usage.py**: 다양한 사용 예제 및 테스트 코드
- **README.md**: 사용법 및 설명서

## 주요 기능

1. **엔코더 펄스 카운팅**: 정확한 위치 추적
2. **회전 방향 감지**: 정방향/역방향 자동 감지
3. **속도 제어**: Enable 핀 PWM을 통한 정밀한 속도 제어 (0-100%)
4. **위치 제어**: 목표 위치로의 자동 이동
5. **실시간 상태 모니터링**: 위치, 방향, RPM 정보 제공

## 주의사항

1. **전원 공급**: 모터에 충분한 전원을 공급하세요
2. **배선 확인**: 연결을 다시 한번 확인하세요
3. **인터럽트 핀**: Phase B는 인터럽트 핀으로 사용됩니다
4. **PWM 주파수**: 15000Hz로 설정되어 있습니다
5. **Enable 핀 점퍼**: L298N의 Enable 핀 점퍼를 제거하세요

## 문제 해결

### 엔코더가 작동하지 않는 경우
- 배선 연결 확인
- 풀업 저항 사용 확인
- 인터럽트 핀 설정 확인

### 모터가 회전하지 않는 경우
- 전원 공급 확인
- L298N 드라이버 연결 확인
- PWM 신호 확인

### 위치가 부정확한 경우
- 엔코더 분해능 확인 (회전당 펄스 수)
- 노이즈 차단을 위한 필터링 추가 고려