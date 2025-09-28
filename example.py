"""
FIT0405 모터 엔코더 라이브러리 사용 예제
fit0405_motor.py를 임포트하여 사용하는 통합 제어 프로그램
"""

from fit0405_motor import FIT0405Motor, L298NDriver, SJ01Encoder, PID
import time


def basic_motor_control():
    """기본 모터 제어 예제"""
    print("=== 기본 모터 제어 예제 ===")
    
    # FIT0405 모터 객체 생성
    motor = FIT0405Motor()
    
    print("정방향 회전 (3초)")
    motor.forward(50)  # 50% 속도로 정방향 회전
    time.sleep(3)
    
    print("역방향 회전 (3초)")
    motor.backward(50)  # 50% 속도로 역방향 회전
    time.sleep(3)
    
    print("정지")
    motor.stop()


def encoder_only_test():
    """엔코더만 사용하는 테스트"""
    print("=== 엔코더만 사용 테스트 ===")
    
    # SJ01 엔코더만 사용
    encoder = SJ01Encoder(pin_a=18, pin_b=17)
    
    print("엔코더를 수동으로 회전시켜보세요...")
    
    for i in range(10):
        pulse_count = encoder.get_pulse_count()
        direction = encoder.get_direction()
        rpm = encoder.get_current_rpm()
        
        print(f"펄스: {pulse_count}, 방향: {'정방향' if direction else '역방향'}, RPM: {rpm:.1f}")
        time.sleep(1)
    
    encoder.reset_count()
    print("엔코더 카운터 리셋됨")


def motor_driver_only_test():
    """모터 드라이버만 사용하는 테스트"""
    print("=== 모터 드라이버만 사용 테스트 ===")
    
    # L298N 드라이버만 사용
    motor_driver = L298NDriver(in1_pin=4, in2_pin=5, enable_pin=6)
    
    print("정방향 회전 (2초)")
    motor_driver.forward(70)  # 70% 속도
    time.sleep(2)
    
    print("역방향 회전 (2초)")
    motor_driver.backward(70)  # 70% 속도
    time.sleep(2)
    
    print("정지")
    motor_driver.stop()


def position_control_example():
    """위치 제어 예제"""
    print("=== 위치 제어 예제 ===")
    
    motor = FIT0405Motor()
    motor.reset_position()
    
    # 목표 위치로 이동
    target_positions = [100, -50, 200, 0, 150]
    
    for target in target_positions:
        print(f"목표 위치 {target}로 이동")
        
        while not motor.move_to_position(target, speed=40, tolerance=5):
            current_pos = motor.get_position()
            print(f"현재 위치: {current_pos}, 목표: {target}")
            time.sleep(0.1)
        
        print(f"목표 위치 {target} 도달!")
        time.sleep(1)
    
    print(f"위치 제어 완료 - 최종 위치: {motor.get_position()}")


def pid_position_control_example():
    """PID 위치 제어 예제"""
    print("=== PID 위치 제어 예제 ===")
    
    motor = FIT0405Motor()
    motor.reset_position()
    
    # 튜닝된 최적 PID 파라미터 설정
    motor.set_position_pid_parameters(kp=0.1, ki=0.1, kd=0.1)
    
    # 여러 목표 위치로 이동
    target_positions = [100, -50, 200, 0, 150, -100]
    
    for target in target_positions:
        print(f"목표 위치: {target}로 PID 제어 이동")
        
        start_time = time.time()
        while not motor.move_to_position_pid(target, tolerance=3):
            current_pos = motor.get_position()
            elapsed = time.time() - start_time
            
            print(f"시간: {elapsed:.1f}s, 현재 위치: {current_pos}, 목표: {target}")
            time.sleep(0.1)
            
            # 타임아웃 체크 (10초)
            if elapsed > 10:
                print("타임아웃 발생")
                break
        
        print(f"목표 위치 {target} 도달! 최종 위치: {motor.get_position()}")
        time.sleep(2)
    
    motor.stop()


def speed_control_example():
    """속도 제어 예제"""
    print("=== 속도 제어 예제 ===")
    
    motor = FIT0405Motor()
    motor.reset_position()
    
    # 다양한 속도로 테스트
    speeds = [20, 40, 60, 80, 100]
    
    for speed in speeds:
        print(f"속도 {speed}%로 3초간 회전")
        motor.forward(speed)
        
        # 3초간 RPM 측정
        for i in range(6):
            time.sleep(0.5)
            position = motor.get_position()
            rpm = motor.get_current_rpm()
            direction = "정방향" if motor.get_direction() else "역방향"
            
            print(f"  시간: {i*0.5:.1f}s, 위치: {position}, RPM: {rpm:.1f}, 방향: {direction}")
        
        motor.stop()
        time.sleep(1)
    
    print("속도 제어 테스트 완료")


def pid_speed_control_example():
    """PID 속도 제어 예제"""
    print("=== PID 속도 제어 예제 ===")
    
    motor = FIT0405Motor()
    motor.reset_position()
    
    # PID 파라미터 설정
    motor.set_speed_pid_parameters(kp=3.0, ki=0.0, kd=0.0)
    
    # 다양한 목표 RPM으로 테스트
    target_rpms = [50, 100, 150, 200, 100, 0]
    
    for target_rpm in target_rpms:
        if target_rpm == 0:
            print("모터 정지")
            motor.stop()
            time.sleep(2)
            continue
        
        print(f"목표 RPM: {target_rpm}으로 PID 제어")
        
        # 5초간 목표 RPM 유지
        for i in range(50):  # 0.1초 간격으로 5초
            current_rpm = motor.control_speed_pid(target_rpm)
            
            if i % 10 == 0:  # 1초마다 출력
                print(f"시간: {i*0.1:.1f}s, 현재 RPM: {current_rpm:.1f}, 목표: {target_rpm}")
            
            time.sleep(0.1)
    
    motor.stop()
    print("PID 속도 제어 완료")


def rpm_monitoring_example():
    """RPM 모니터링 예제"""
    print("=== RPM 모니터링 예제 ===")
    
    motor = FIT0405Motor()
    motor.reset_position()
    
    print("정방향으로 10초간 회전하면서 실시간 RPM 측정")
    motor.forward(60)  # 60% 속도
    
    start_time = time.time()
    
    while time.time() - start_time < 10:
        current_time = time.time() - start_time
        position = motor.get_position()
        rpm = motor.get_current_rpm()
        direction = "정방향" if motor.get_direction() else "역방향"
        
        print(f"시간: {current_time:.1f}s, 위치: {position}, RPM: {rpm:.1f}, 방향: {direction}")
        time.sleep(0.5)
    
    motor.stop()
    print("RPM 모니터링 완료")


def speed_ramp_example():
    """속도 램프 예제 - 점진적 속도 증가/감소"""
    print("=== 속도 램프 예제 ===")
    
    motor = FIT0405Motor()
    motor.reset_position()
    
    print("정방향으로 속도 점진적 증가 (0% → 100%)")
    
    # 속도 점진적 증가
    for speed in range(0, 101, 10):
        print(f"속도: {speed}%")
        motor.forward(speed)
        
        # 1초간 측정
        time.sleep(1)
        
        position = motor.get_position()
        rpm = motor.get_current_rpm()
        print(f"  위치: {position}, RPM: {rpm:.1f}")
    
    print("속도 점진적 감소 (100% → 0%)")
    
    # 속도 점진적 감소
    for speed in range(100, -1, -10):
        print(f"속도: {speed}%")
        motor.forward(speed)
        
        # 1초간 측정
        time.sleep(1)
        
        position = motor.get_position()
        rpm = motor.get_current_rpm()
        print(f"  위치: {position}, RPM: {rpm:.1f}")
    
    motor.stop()
    print("속도 램프 테스트 완료")


def pid_tuning_example():
    """PID 튜닝 예제"""
    print("=== PID 튜닝 예제 ===")
    
    motor = FIT0405Motor()
    motor.reset_position()
    
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
        motor.set_position_pid_parameters(
            kp=params["kp"], 
            ki=params["ki"], 
            kd=params["kd"]
        )
        
        # 목표 위치로 이동 테스트
        target = 100
        motor.reset_position()
        
        start_time = time.time()
        while not motor.move_to_position_pid(target, tolerance=5):
            current_pos = motor.get_position()
            elapsed = time.time() - start_time
            
            if elapsed > 5:  # 5초 타임아웃
                print("타임아웃")
                break
            
            time.sleep(0.1)
        
        final_pos = motor.get_position()
        error = abs(target - final_pos)
        print(f"최종 위치: {final_pos}, 오차: {error}")
        
        time.sleep(1)
    
    motor.stop()
    print("PID 튜닝 테스트 완료")


def hardware_test_example():
    """하드웨어 테스트 예제"""
    print("=== 하드웨어 테스트 예제 ===")
    
    motor = FIT0405Motor()
    
    print("1. 모터 드라이버 테스트")
    motor.motor.forward(50)
    time.sleep(2)
    motor.motor.backward(50)
    time.sleep(2)
    motor.motor.stop()
    
    print("2. 엔코더 테스트")
    print("엔코더를 수동으로 회전시켜보세요...")
    for i in range(5):
        position = motor.get_position()
        direction = motor.get_direction()
        print(f"위치: {position}, 방향: {'정방향' if direction else '역방향'}")
        time.sleep(1)
    
    print("3. 통합 테스트")
    motor.reset_position()
    motor.forward(30)
    time.sleep(3)
    print(f"통합 테스트 완료 - 최종 위치: {motor.get_position()}")
    motor.stop()
    
    print("하드웨어 테스트 완료")


def safe_motor_test():
    """안전한 모터 제어 테스트"""
    print("=== 안전한 모터 제어 테스트 ===")
    
    motor_driver = L298NDriver(in1_pin=4, in2_pin=5, enable_pin=6)
    
    print("안전한 정방향 회전 (3초)")
    motor_driver.safe_forward(70)  # 70% 속도 (duty cycle 제한됨)
    time.sleep(3)
    
    print("안전한 역방향 회전 (3초)")
    motor_driver.safe_backward(70)  # 70% 속도 (duty cycle 제한됨)
    time.sleep(3)
    
    print("정지")
    motor_driver.stop()
    
    print("안전한 모터 제어 완료")


def duty_cycle_test():
    """Duty cycle 범위 테스트"""
    print("=== Duty Cycle 범위 테스트 ===")
    
    motor_driver = L298NDriver(in1_pin=4, in2_pin=5, enable_pin=6)
    
    # 다양한 속도로 duty cycle 확인
    speeds = [10, 25, 50, 75, 100]
    
    for speed in speeds:
        duty = motor_driver.duty_cycle(speed)
        print(f"속도 {speed}% → Duty Cycle: {duty}")
        
        # 각 속도로 1초간 테스트
        print(f"  정방향 테스트...")
        motor_driver.safe_forward(speed)
        time.sleep(1)
        
        print(f"  역방향 테스트...")
        motor_driver.safe_backward(speed)
        time.sleep(1)
        
        motor_driver.stop()
        time.sleep(0.5)
    
    print("Duty cycle 테스트 완료")


def evaluate_pid_performance(motor, target_positions, timeout=8):
    """PID 성능 평가 함수"""
    total_error = 0
    overshoot_count = 0
    settling_times = []
    success_count = 0
    
    print("  성능 평가 중...")
    
    for i, target in enumerate(target_positions):
        print(f"    목표 {i+1}/{len(target_positions)}: {target}")
        
        motor.reset_position()
        start_time = time.time()
        max_position = 0
        min_position = 0
        
        # 목표 위치로 이동
        while True:
            current_pos = motor.get_position()
            
            # 최대/최소 위치 추적 (오버슈트 감지용)
            max_position = max(max_position, current_pos)
            min_position = min(min_position, current_pos)
            
            # 목표 도달 체크
            if abs(target - current_pos) <= 3:  # 허용 오차 3
                settling_time = time.time() - start_time
                settling_times.append(settling_time)
                success_count += 1
                
                final_pos = motor.get_position()
                error = abs(target - final_pos)
                total_error += error
                
                print(f"      도달! 위치: {final_pos}, 오차: {error:.1f}, 시간: {settling_time:.1f}s")
                break
            
            # 타임아웃 체크
            if time.time() - start_time > timeout:
                final_pos = motor.get_position()
                error = abs(target - final_pos)
                total_error += error
                settling_times.append(timeout)
                
                print(f"      타임아웃! 위치: {final_pos}, 오차: {error:.1f}")
                break
            
            # PID 제어 실행
            if not motor.move_to_position_pid(target, tolerance=3):
                time.sleep(0.1)
        
        # 오버슈트 체크
        if target > 0 and max_position > target * 1.15:  # 15% 이상 오버슈트
            overshoot_count += 1
        elif target < 0 and min_position < target * 1.15:
            overshoot_count += 1
        
        time.sleep(0.5)  # 다음 테스트 전 대기
    
    # 결과 계산
    if settling_times:
        avg_settling_time = sum(settling_times) / len(settling_times)
    else:
        avg_settling_time = timeout
    
    avg_error = total_error / len(target_positions)
    success_rate = success_count / len(target_positions)
    overshoot_rate = overshoot_count / len(target_positions)
    
    return {
        'average_error': avg_error,
        'success_rate': success_rate,
        'overshoot_rate': overshoot_rate,
        'average_settling_time': avg_settling_time
    }


def auto_tune_position_pid():
    """자동 위치 제어 PID 튜닝"""
    print("=== 자동 위치 제어 PID 튜닝 시작 ===")
    
    # 모터 객체 생성
    motor = FIT0405Motor()
    
    # 테스트 목표 위치들
    test_targets = [50, 100, -50, 200, -100, 0]
    
    best_params = None
    best_score = float('inf')
    test_results = []
    
    # Kp 범위 테스트
    kp_range = [0.1, 0.3, 0.5, 0.8, 1.0, 1.5, 2.0]
    ki_range = [0.0, 0.05, 0.1, 0.2, 0.3]
    kd_range = [0.0, 0.05, 0.1, 0.2, 0.3]
    
    total_tests = len(kp_range) * len(ki_range) * len(kd_range)
    current_test = 0
    
    for kp in kp_range:
        for ki in ki_range:
            for kd in kd_range:
                current_test += 1
                print(f"\n테스트 {current_test}/{total_tests}: Kp={kp}, Ki={ki}, Kd={kd}")
                
                # PID 파라미터 설정
                motor.set_position_pid_parameters(kp=kp, ki=ki, kd=kd)
                
                # 성능 평가
                performance = evaluate_pid_performance(motor, test_targets)
                
                # 점수 계산 (가중치 적용)
                score = (performance['average_error'] * 2.0 +           # 오차 가중치
                        (1 - performance['success_rate']) * 50.0 +     # 성공률 가중치
                        performance['overshoot_rate'] * 20.0 +         # 오버슈트 가중치
                        performance['average_settling_time'] * 0.5)    # 시간 가중치
                
                print(f"  결과: 오차={performance['average_error']:.1f}, "
                      f"성공률={performance['success_rate']:.1%}, "
                      f"오버슈트={performance['overshoot_rate']:.1%}, "
                      f"시간={performance['average_settling_time']:.1f}s, "
                      f"점수={score:.2f}")
                
                # 결과 저장
                test_results.append({
                    'params': {'kp': kp, 'ki': ki, 'kd': kd},
                    'performance': performance,
                    'score': score
                })
                
                if score < best_score:
                    best_score = score
                    best_params = {'kp': kp, 'ki': ki, 'kd': kd}
                    print(f"  ★ 새로운 최적 파라미터!")
    
    # 결과 정리
    print(f"\n=== 튜닝 완료 ===")
    print(f"최적 파라미터: Kp={best_params['kp']}, Ki={best_params['ki']}, Kd={best_params['kd']}")
    print(f"최적 점수: {best_score:.2f}")
    
    # 상위 5개 결과 출력
    test_results.sort(key=lambda x: x['score'])
    print(f"\n상위 5개 결과:")
    for i, result in enumerate(test_results[:5]):
        params = result['params']
        perf = result['performance']
        print(f"{i+1}. Kp={params['kp']}, Ki={params['ki']}, Kd={params['kd']} "
              f"(점수: {result['score']:.2f}, 오차: {perf['average_error']:.1f})")
    
    # 최적 파라미터 적용
    motor.set_position_pid_parameters(
        kp=best_params['kp'], 
        ki=best_params['ki'], 
        kd=best_params['kd']
    )
    
    return best_params, test_results


def auto_tune_speed_pid():
    """자동 속도 제어 PID 튜닝"""
    print("=== 자동 속도 제어 PID 튜닝 시작 ===")
    
    # 모터 객체 생성
    motor = FIT0405Motor()
    
    # 테스트 목표 RPM들
    test_targets = [50, 100, 150, 200, 100, 0]
    
    best_params = None
    best_score = float('inf')
    test_results = []
    
    # 속도 제어용 파라미터 범위
    kp_range = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
    ki_range = [0.0, 0.05, 0.1, 0.2, 0.3]
    kd_range = [0.0, 0.05, 0.1, 0.2]
    
    total_tests = len(kp_range) * len(ki_range) * len(kd_range)
    current_test = 0
    
    for kp in kp_range:
        for ki in ki_range:
            for kd in kd_range:
                current_test += 1
                print(f"\n테스트 {current_test}/{total_tests}: Kp={kp}, Ki={ki}, Kd={kd}")
                
                # PID 파라미터 설정
                motor.set_speed_pid_parameters(kp=kp, ki=ki, kd=kd)
                
                # 성능 평가
                performance = evaluate_speed_performance(motor, test_targets)
                
                # 점수 계산
                score = (performance['average_error'] * 1.0 +           # RPM 오차
                        (1 - performance['stability']) * 30.0 +        # 안정성
                        performance['overshoot_rate'] * 15.0)          # 오버슈트
                
                print(f"  결과: 오차={performance['average_error']:.1f}RPM, "
                      f"안정성={performance['stability']:.1%}, "
                      f"오버슈트={performance['overshoot_rate']:.1%}, "
                      f"점수={score:.2f}")
                
                test_results.append({
                    'params': {'kp': kp, 'ki': ki, 'kd': kd},
                    'performance': performance,
                    'score': score
                })
                
                if score < best_score:
                    best_score = score
                    best_params = {'kp': kp, 'ki': ki, 'kd': kd}
                    print(f"  ★ 새로운 최적 파라미터!")
    
    print(f"\n=== 속도 제어 튜닝 완료 ===")
    print(f"최적 파라미터: Kp={best_params['kp']}, Ki={best_params['ki']}, Kd={best_params['kd']}")
    
    return best_params, test_results


def evaluate_speed_performance(motor, target_rpms, test_duration=3):
    """속도 제어 성능 평가"""
    total_error = 0
    overshoot_count = 0
    stability_measurements = []
    
    for target_rpm in target_rpms:
        if target_rpm == 0:
            motor.stop()
            time.sleep(1)
            continue
        
        print(f"    목표 RPM: {target_rpm}")
        motor.reset_position()
        
        # 목표 RPM으로 제어
        rpm_measurements = []
        start_time = time.time()
        
        while time.time() - start_time < test_duration:
            current_rpm = motor.control_speed_pid(target_rpm)
            rpm_measurements.append(current_rpm)
            time.sleep(0.1)
        
        # 성능 분석
        if rpm_measurements:
            avg_rpm = sum(rpm_measurements) / len(rpm_measurements)
            error = abs(target_rpm - avg_rpm)
            total_error += error
            
            # 안정성 계산 (표준편차 기반)
            variance = sum((rpm - avg_rpm) ** 2 for rpm in rpm_measurements) / len(rpm_measurements)
            stability = max(0, 1 - (variance ** 0.5) / target_rpm)  # 0-1 범위
            
            # 오버슈트 체크
            max_rpm = max(rpm_measurements)
            if max_rpm > target_rpm * 1.2:  # 20% 이상 오버슈트
                overshoot_count += 1
            
            stability_measurements.append(stability)
            print(f"      평균 RPM: {avg_rpm:.1f}, 오차: {error:.1f}, 안정성: {stability:.1%}")
        
        time.sleep(0.5)
    
    # 결과 계산
    avg_error = total_error / len([rpm for rpm in target_rpms if rpm > 0])
    avg_stability = sum(stability_measurements) / len(stability_measurements) if stability_measurements else 0
    overshoot_rate = overshoot_count / len([rpm for rpm in target_rpms if rpm > 0])
    
    return {
        'average_error': avg_error,
        'stability': avg_stability,
        'overshoot_rate': overshoot_rate
    }


def quick_pid_test():
    """빠른 PID 테스트 (제한된 파라미터로)"""
    print("=== 빠른 PID 테스트 ===")
    
    # 모터 객체 생성
    motor = FIT0405Motor()
    
    # 제한된 파라미터 범위로 빠른 테스트
    quick_params = [
        {'kp': 0.5, 'ki': 0.0, 'kd': 0.0, 'name': 'P만'},
        {'kp': 0.5, 'ki': 0.1, 'kd': 0.0, 'name': 'PI'},
        {'kp': 0.5, 'ki': 0.1, 'kd': 0.1, 'name': 'PID'},
        {'kp': 1.0, 'ki': 0.1, 'kd': 0.1, 'name': '강화된 PID'},
        {'kp': 0.8, 'ki': 0.05, 'kd': 0.2, 'name': '안정적 PID'}
    ]
    
    test_targets = [100, -50, 200]
    best_params = None
    best_score = float('inf')
    
    for params in quick_params:
        print(f"\n테스트: {params['name']} (Kp={params['kp']}, Ki={params['ki']}, Kd={params['kd']})")
        
        motor.set_position_pid_parameters(kp=params['kp'], ki=params['ki'], kd=params['kd'])
        performance = evaluate_pid_performance(motor, test_targets, timeout=5)
        
        score = (performance['average_error'] * 2.0 + 
                (1 - performance['success_rate']) * 50.0 + 
                performance['overshoot_rate'] * 20.0)
        
        print(f"  점수: {score:.2f}")
        
        if score < best_score:
            best_score = score
            best_params = params
    
    print(f"\n빠른 테스트 최적: {best_params['name']}")
    return best_params

def measure_motor_max_performance():
    """모터 최대 성능 측정 (기준점 설정) - 개선된 버전"""
    print("=== 모터 최대 성능 측정 (개선된 버전) ===")
    
    motor = FIT0405Motor()
    
    print("1. 최대 PWM에서 최대 RPM 측정")
    print("2. 피크 도달 후 안정된 상태에서 측정")
    
    # 최대 PWM으로 모터 구동
    motor.reset_position()
    motor.forward(100)  # 100% 속도
    
    # 가속 구간 대기 및 피크 찾기
    print("가속 중... 피크 RPM 찾는 중...")
    rpm_measurements = []
    peak_rpm = 0
    peak_time = 0
    stable_count = 0
    
    start_time = time.time()
    
    # 피크 도달까지 측정 (최대 5초)
    while time.time() - start_time < 5:
        current_rpm = motor.get_current_rpm()
        elapsed = time.time() - start_time
        
        rpm_measurements.append(current_rpm)
        
        # 피크 RPM 업데이트
        if current_rpm > peak_rpm:
            peak_rpm = current_rpm
            peak_time = elapsed
            stable_count = 0
        elif current_rpm >= peak_rpm * 0.95:  # 피크의 95% 이상
            stable_count += 1
        else:
            stable_count = 0
        
        # 0.5초마다 진행 상황 출력
        if int(elapsed * 2) % 1 == 0 and elapsed - int(elapsed) < 0.1:
            print(f"  시간: {elapsed:.1f}s, 현재 RPM: {current_rpm:.1f}, 피크: {peak_rpm:.1f}")
        
        # 안정된 상태 확인 (피크의 95% 이상을 1초간 유지)
        if stable_count >= 10:  # 1초간 안정 (0.1초 * 10)
            print(f"피크 도달! 피크 RPM: {peak_rpm:.1f} (시간: {elapsed:.1f}s)")
            break
        
        time.sleep(0.1)
    
    # 피크 도달 후 안정된 상태에서 10초간 측정
    print(f"피크 도달 후 10초간 안정된 상태 측정...")
    stable_rpm_measurements = []
    stable_pulse_measurements = []
    
    measurement_start_time = time.time()
    initial_pulses = motor.get_position()
    
    while time.time() - measurement_start_time < 10:
        current_rpm = motor.get_current_rpm()
        current_pulses = motor.get_position()
        
        # 0.0 RPM 쓰레기 값 실시간 필터링
        if current_rpm > 0.1:  # 0.1 RPM 이상만 유효한 값으로 간주
            stable_rpm_measurements.append(current_rpm)
            stable_pulse_measurements.append(current_pulses)
        
        # 2초마다 진행 상황 출력 (유효한 값만)
        elapsed = time.time() - measurement_start_time
        if int(elapsed) % 2 == 0 and elapsed - int(elapsed) < 0.1:
            if current_rpm > 0.1:
                print(f"  측정 시간: {elapsed:.1f}s, 현재 RPM: {current_rpm:.1f}")
            else:
                print(f"  측정 시간: {elapsed:.1f}s, 현재 RPM: {current_rpm:.1f} (필터링됨)")
        
        time.sleep(0.1)
    
    motor.stop()
    
    # 결과 분석
    if stable_rpm_measurements and stable_pulse_measurements:
        # 0.0 RPM 쓰레기 값 필터링
        valid_rpm_measurements = [rpm for rpm in stable_rpm_measurements if rpm > 0.1]
        valid_pulse_measurements = [pulse for pulse in stable_pulse_measurements if pulse != 0]
        
        print(f"\n=== 데이터 필터링 결과 ===")
        print(f"전체 측정값: {len(stable_rpm_measurements)}개")
        print(f"유효 측정값: {len(valid_rpm_measurements)}개")
        print(f"필터링된 값: {len(stable_rpm_measurements) - len(valid_rpm_measurements)}개")
        
        if valid_rpm_measurements:
            max_rpm = max(valid_rpm_measurements)
            avg_rpm = sum(valid_rpm_measurements) / len(valid_rpm_measurements)
            min_rpm = min(valid_rpm_measurements)
            
            final_pulses = motor.get_position()
            total_pulses = final_pulses - initial_pulses
            
            print(f"\n=== 측정 결과 (필터링 적용) ===")
            print(f"피크 RPM: {peak_rpm:.1f} (도달 시간: {peak_time:.1f}s)")
            print(f"안정 상태 최대 RPM: {max_rpm:.1f}")
            print(f"안정 상태 평균 RPM: {avg_rpm:.1f}")
            print(f"안정 상태 최소 RPM: {min_rpm:.1f}")
            print(f"RPM 변동 범위: {min_rpm:.1f} ~ {max_rpm:.1f}")
            print(f"RPM 표준편차: {((sum((rpm - avg_rpm)**2 for rpm in valid_rpm_measurements) / len(valid_rpm_measurements))**0.5):.2f}")
            print(f"총 펄스 수: {total_pulses}")
            print(f"안정 상태 측정 시간: 10초")
            
            # 펄스 기반 RPM 계산 (1920 PPR 기준)
            pulses_per_second = total_pulses / 10
            calculated_rpm = (pulses_per_second / 1920) * 60
            print(f"펄스 기반 계산 RPM: {calculated_rpm:.1f}")
            
            # 기준점 설정 (안정된 평균 RPM 사용)
            motor_max_rpm = avg_rpm
            print(f"\n=== 기준점 설정 ===")
            print(f"모터 최대 RPM (안정 상태): {motor_max_rpm:.1f}")
            print(f"권장 목표 RPM: {motor_max_rpm * 0.3:.1f}, {motor_max_rpm * 0.5:.1f}, {motor_max_rpm * 0.7:.1f}, {motor_max_rpm * 0.9:.1f}")
            
            return motor_max_rpm
        else:
            print("유효한 RPM 측정값이 없습니다!")
            return None
    
    return None

def realistic_speed_control_test():
    """현실적인 속도 제어 테스트 (측정된 최대 RPM 기반)"""
    print("=== 현실적인 속도 제어 테스트 ===")
    
    # 먼저 모터 최대 성능 측정
    max_rpm = measure_motor_max_performance()
    
    if max_rpm is None:
        print("모터 성능 측정 실패")
        return
    
    motor = FIT0405Motor()
    
    # 측정된 최대 RPM을 기준으로 현실적인 목표 설정
    realistic_targets = [
        max_rpm * 0.3,  # 30%
        max_rpm * 0.5,  # 50%
        max_rpm * 0.7,  # 70%
        max_rpm * 0.9   # 90%
    ]
    
    print(f"\n=== 현실적인 목표 RPM으로 테스트 ===")
    print(f"기준 최대 RPM: {max_rpm:.1f}")
    
    # 개선된 PID 파라미터
    motor.set_speed_pid_parameters(kp=2.0, ki=0.1, kd=0.1)
    
    for target_rpm in realistic_targets:
        print(f"\n=== 목표 RPM: {target_rpm:.1f} ({target_rpm/max_rpm*100:.0f}% of max) ===")
        
        motor.reset_position()
        
        # 5초간 속도 제어
        rpm_measurements = []
        start_time = time.time()
        
        while time.time() - start_time < 5:
            current_rpm = motor.control_speed_pid(target_rpm)
            rpm_measurements.append(current_rpm)
            time.sleep(0.1)
        
        # 결과 분석
        if rpm_measurements:
            avg_rpm = sum(rpm_measurements) / len(rpm_measurements)
            max_measured = max(rpm_measurements)
            min_measured = min(rpm_measurements)
            error = abs(target_rpm - avg_rpm)
            success_rate = (1 - error / target_rpm) * 100
            
            print(f"목표: {target_rpm:.1f} RPM")
            print(f"달성: {avg_rpm:.1f} RPM")
            print(f"오차: {error:.1f} RPM")
            print(f"성공률: {success_rate:.1f}%")
            print(f"변동 범위: {min_measured:.1f} ~ {max_measured:.1f} RPM")
        
        time.sleep(1)
    
    motor.stop()
    print("현실적인 속도 제어 테스트 완료")

def encoder_pulse_analysis():
    """엔코더 펄스 분석"""
    print("=== 엔코더 펄스 분석 ===")
    
    motor = FIT0405Motor()
    
    print("엔코더를 수동으로 회전시켜보세요...")
    print("10초간 측정합니다.")
    
    initial_pulses = motor.get_position()
    start_time = time.time()
    
    while time.time() - start_time < 10:
        current_pulses = motor.get_position()
        elapsed = time.time() - start_time
        
        if int(elapsed) % 2 == 0 and elapsed - int(elapsed) < 0.1:
            print(f"시간: {elapsed:.1f}s, 펄스: {current_pulses}")
        
        time.sleep(0.1)
    
    final_pulses = motor.get_position()
    total_pulses = final_pulses - initial_pulses
    
    print(f"\n=== 엔코더 분석 결과 ===")
    print(f"초기 펄스: {initial_pulses}")
    print(f"최종 펄스: {final_pulses}")
    print(f"총 펄스 변화: {total_pulses}")
    print(f"측정 시간: 10초")
    print(f"초당 펄스: {total_pulses / 10:.1f}")
    
    if total_pulses > 0:
        # 1920 PPR 기준으로 RPM 계산
        rpm = (total_pulses / 10 / 1920) * 60
        print(f"계산된 RPM: {rpm:.1f}")
    
    print("엔코더 펄스 분석 완료")

def pid_speed_control_test(kp=0.3, ki=2.0, kd=0.1, target_rpm=75.0, test_duration=10):
    """PID 속도 제어 테스트 (파라미터 수정 가능)"""
    print(f"=== PID 속도 제어 테스트 ===")
    print(f"파라미터: Kp={kp}, Ki={ki}, Kd={kd}")
    print(f"목표 RPM: {target_rpm}, 테스트 시간: {test_duration}초")
    
    motor = FIT0405Motor()
    motor.set_speed_pid_parameters(kp=kp, ki=ki, kd=kd)
    
    motor.reset_position()
    
    # 속도 제어
    rpm_measurements = []
    start_time = time.time()
    
    print("속도 제어 시작...")
    
    while time.time() - start_time < test_duration:
        current_rpm = motor.control_speed_pid(target_rpm)
        
        if current_rpm > 0.1:  # 쓰레기 값 필터링
            rpm_measurements.append(current_rpm)
        
        # 1초마다 진행 상황 출력
        elapsed = time.time() - start_time
        if int(elapsed) % 1 == 0 and elapsed - int(elapsed) < 0.1:
            if current_rpm > 0.1:
                print(f"  시간: {elapsed:.1f}s, 목표: {target_rpm}, 현재: {current_rpm:.1f} RPM")
        
        time.sleep(0.1)
    
    motor.stop()
    
    # 결과 분석
    if rpm_measurements:
        avg_rpm = sum(rpm_measurements) / len(rpm_measurements)
        max_rpm = max(rpm_measurements)
        min_rpm = min(rpm_measurements)
        error = abs(target_rpm - avg_rpm)
        success_rate = (1 - error / target_rpm) * 100
        
        # 표준편차 계산
        variance = sum((rpm - avg_rpm)**2 for rpm in rpm_measurements) / len(rpm_measurements)
        std_dev = variance**0.5
        
        print(f"\n=== 결과 ===")
        print(f"목표 RPM: {target_rpm}")
        print(f"달성 RPM: {avg_rpm:.1f}")
        print(f"오차: {error:.1f} RPM")
        print(f"성공률: {success_rate:.1f}%")
        print(f"변동 범위: {min_rpm:.1f} ~ {max_rpm:.1f} RPM")
        print(f"표준편차: {std_dev:.2f} RPM")
        
        # 성능 평가
        if success_rate >= 95:
            print("성능: 매우 우수 (95% 이상)")
        elif success_rate >= 90:
            print("성능: 우수 (90% 이상)")
        elif success_rate >= 80:
            print("성능: 양호 (80% 이상)")
        else:
            print("성능: 개선 필요 (80% 미만)")
        
        return {
            "target_rpm": target_rpm,
            "achieved_rpm": avg_rpm,
            "error": error,
            "success_rate": success_rate,
            "std_dev": std_dev
        }
    else:
        print("유효한 측정값이 없습니다!")
        return None


def pid_position_control_test(kp=0.1, ki=0.1, kd=0.1, target_position=100, test_duration=8):
    """PID 위치 제어 테스트 (파라미터 수정 가능)"""
    print(f"=== PID 위치 제어 테스트 ===")
    print(f"파라미터: Kp={kp}, Ki={ki}, Kd={kd}")
    print(f"목표 위치: {target_position}, 테스트 시간: {test_duration}초")
    
    motor = FIT0405Motor()
    motor.set_position_pid_parameters(kp=kp, ki=ki, kd=kd)
    
    motor.reset_position()
    
    # 위치 제어
    position_measurements = []
    start_time = time.time()
    
    print("위치 제어 시작...")
    
    while time.time() - start_time < test_duration:
        current_position = motor.control_position_pid(target_position)
        position_measurements.append(current_position)
        
        # 1초마다 진행 상황 출력
        elapsed = time.time() - start_time
        if int(elapsed) % 1 == 0 and elapsed - int(elapsed) < 0.1:
            print(f"  시간: {elapsed:.1f}s, 목표: {target_position}, 현재: {current_position}")
        
        time.sleep(0.1)
    
    motor.stop()
    
    # 결과 분석
    if position_measurements:
        final_position = position_measurements[-1]
        error = abs(target_position - final_position)
        success_rate = (1 - error / abs(target_position)) * 100 if target_position != 0 else 100
        
        print(f"\n=== 결과 ===")
        print(f"목표 위치: {target_position}")
        print(f"달성 위치: {final_position}")
        print(f"오차: {error}")
        print(f"성공률: {success_rate:.1f}%")
        
        # 성능 평가
        if success_rate >= 95:
            print("성능: 매우 우수 (95% 이상)")
        elif success_rate >= 90:
            print("성능: 우수 (90% 이상)")
        elif success_rate >= 80:
            print("성능: 양호 (80% 이상)")
        else:
            print("성능: 개선 필요 (80% 미만)")
        
        return {
            "target_position": target_position,
            "achieved_position": final_position,
            "error": error,
            "success_rate": success_rate
        }
    else:
        print("유효한 측정값이 없습니다!")
        return None


def main():
    """메인 함수 - 실행할 예제 선택"""
    print("FIT0405 모터 엔코더 라이브러리 예제 프로그램")
    print("=" * 50)
    
    # 기본 예제 실행
    basic_motor_control()
    time.sleep(2)
    
    # 다른 예제들을 실행하려면 아래 주석 해제
    # encoder_only_test()
    # motor_driver_only_test()
    # position_control_example()
    # pid_position_control_example()
    # improved_pid_position_control()  # 개선된 PID 제어
    # test_different_pid_params()  # 다양한 PID 파라미터 테스트
    # speed_control_example()
    # pid_speed_control_example()
    # rpm_monitoring_example()
    # speed_ramp_example()
    # pid_tuning_example()
    # hardware_test_example()
    # safe_motor_test()
    # duty_cycle_test()
    
    # 자동 PID 튜닝 예제들
    # quick_pid_test()  # 빠른 테스트
    # auto_tune_position_pid()  # 전체 위치 제어 튜닝
    # auto_tune_speed_pid()  # 전체 속도 제어 튜닝
    
    # 모터 성능 측정 및 현실적인 제어 테스트
    # measure_motor_max_performance()  # 모터 최대 성능 측정 (개선된 버전)
    # measure_motor_performance_detailed()  # 모터 성능 상세 측정 (가속/정상/감속 분석)
    # realistic_speed_control_test()  # 현실적인 속도 제어 테스트
    # encoder_pulse_analysis()  # 엔코더 펄스 분석
    
    # 간단한 PID 테스트 (파라미터 수정 가능)
    # pid_speed_control_test(0.3, 2, 0.1, 75, 10)  # 속도 제어: Kp, Ki, Kd, 목표RPM, 시간
    # pid_position_control_test(0.1, 0.1, 0.1, 100, 8)  # 위치 제어: Kp, Ki, Kd, 목표위치, 시간
    # optimize_std_deviation()  # 표준편차 최적화 테스트
    # aggressive_std_optimization()  # 공격적인 표준편차 최적화
    # stability_optimization()  # 안정성 우선 최적화
    
    print("\n모든 예제 실행 완료!")


if __name__ == "__main__":
    main()
