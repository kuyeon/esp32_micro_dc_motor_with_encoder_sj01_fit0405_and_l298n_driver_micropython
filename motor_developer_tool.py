"""
모터 개발자 도구 - 시각화 및 PID 튜닝
FIT0405 모터 + SJ01 엔코더 + L298N 드라이버용
"""

import time
import math
from fit0405_motor import FIT0405Motor

class MotorDeveloperTool:
    def __init__(self):
        self.motor = FIT0405Motor()
        self.data_log = []
        self.is_logging = False
        
    def start_logging(self):
        """데이터 로깅 시작"""
        self.data_log = []
        self.is_logging = True
        print("데이터 로깅 시작")
        
    def stop_logging(self):
        """데이터 로깅 중지"""
        self.is_logging = False
        print(f"데이터 로깅 중지 - 총 {len(self.data_log)}개 데이터 수집")
        
    def log_data(self, timestamp, target_rpm, actual_rpm, kp, ki, kd, position=0):
        """데이터 로깅"""
        if self.is_logging:
            self.data_log.append({
                'time': timestamp,
                'target_rpm': target_rpm,
                'actual_rpm': actual_rpm,
                'kp': kp,
                'ki': ki,
                'kd': kd,
                'position': position
            })
    
    def print_simple_graph(self, data_type='rpm', width=50, height=20):
        """간단한 텍스트 그래프 출력"""
        if not self.data_log:
            print("로그 데이터가 없습니다.")
            return
            
        # 데이터 추출
        if data_type == 'rpm':
            target_data = [d['target_rpm'] for d in self.data_log]
            actual_data = [d['actual_rpm'] for d in self.data_log]
            y_label = "RPM"
        elif data_type == 'position':
            target_data = [d['position'] for d in self.data_log]
            actual_data = [d['position'] for d in self.data_log]
            y_label = "Position"
        else:
            print("지원하지 않는 데이터 타입입니다.")
            return
            
        # 데이터 범위 계산
        all_data = target_data + actual_data
        min_val = min(all_data)
        max_val = max(all_data)
        range_val = max_val - min_val
        
        if range_val == 0:
            print("데이터 범위가 0입니다.")
            return
            
        print(f"\n=== {y_label} 그래프 ===")
        print(f"범위: {min_val:.1f} ~ {max_val:.1f}")
        print(f"시간: 0 ~ {len(self.data_log)*0.1:.1f}초")
        print()
        
        # Y축 레이블
        print(f"{y_label:>8} |", end="")
        for i in range(width):
            print("-", end="")
        print()
        
        # 그래프 그리기
        for row in range(height):
            y_val = max_val - (row * range_val / height)
            print(f"{y_val:>8.1f} |", end="")
            
            for col in range(width):
                if col < len(self.data_log):
                    # 목표값 (빨간색 대신 'T'로 표시)
                    target_y = height - int((target_data[col] - min_val) * height / range_val)
                    # 실제값 (파란색 대신 'A'로 표시)
                    actual_y = height - int((actual_data[col] - min_val) * height / range_val)
                    
                    if row == target_y:
                        print("T", end="")
                    elif row == actual_y:
                        print("A", end="")
                    elif abs(target_y - actual_y) <= 1 and (row == target_y or row == actual_y):
                        print("+", end="")
                    else:
                        print(" ", end="")
                else:
                    print(" ", end="")
            print()
        
        # X축 레이블
        print(f"{'':>8} |", end="")
        for i in range(0, width, 10):
            print(f"{i*0.1:>4.1f}s", end="")
        print()
        
        # 범례
        print("\n범례: T=목표값, A=실제값, +=일치")
        
    def analyze_performance(self):
        """성능 분석"""
        if not self.data_log:
            print("분석할 데이터가 없습니다.")
            return
            
        # RPM 데이터 분석
        target_rpms = [d['target_rpm'] for d in self.data_log]
        actual_rpms = [d['actual_rpm'] for d in self.data_log]
        
        if not actual_rpms:
            print("실제 RPM 데이터가 없습니다.")
            return
            
        # 통계 계산
        avg_target = sum(target_rpms) / len(target_rpms)
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
        
        print(f"\n=== 성능 분석 결과 ===")
        print(f"목표 RPM: {avg_target:.1f}")
        print(f"실제 평균 RPM: {avg_actual:.1f}")
        print(f"실제 최대 RPM: {max_actual:.1f}")
        print(f"실제 최소 RPM: {min_actual:.1f}")
        print(f"평균 오차: {avg_error:.1f} RPM")
        print(f"최대 오차: {max_error:.1f} RPM")
        print(f"표준편차: {std_dev:.2f} RPM")
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
            'avg_target': avg_target,
            'avg_actual': avg_actual,
            'avg_error': avg_error,
            'std_dev': std_dev,
            'success_rate': success_rate
        }
    
    def test_forward_reverse_rpm(self, test_duration=5):
        """정역 RPM 제어 테스트"""
        print("=== 정역 RPM 제어 테스트 ===")
        
        test_speeds = [30, 50, 70, 90]
        
        for speed in test_speeds:
            print(f"\n--- 정방향 {speed}% 테스트 ---")
            
            # 정방향
            self.motor.reset_position()
            self.start_logging()
            start_time = time.time()
            
            self.motor.forward(speed)
            
            while time.time() - start_time < test_duration:
                current_rpm = self.motor.get_current_rpm()
                current_pos = self.motor.get_position()
                elapsed = time.time() - start_time
                
                self.log_data(elapsed, speed, current_rpm, 0, 0, 0, current_pos)
                
                if int(elapsed) % 1 == 0 and elapsed - int(elapsed) < 0.1:
                    print(f"  시간: {elapsed:.1f}s, RPM: {current_rpm:.1f}, 위치: {current_pos}")
                
                time.sleep(0.1)
            
            self.motor.stop()
            self.stop_logging()
            
            # 결과 출력
            self.print_simple_graph('rpm')
            self.analyze_performance()
            
            time.sleep(1)
            
            print(f"\n--- 역방향 {speed}% 테스트 ---")
            
            # 역방향
            self.motor.reset_position()
            self.start_logging()
            start_time = time.time()
            
            self.motor.backward(speed)
            
            while time.time() - start_time < test_duration:
                current_rpm = self.motor.get_current_rpm()
                current_pos = self.motor.get_position()
                elapsed = time.time() - start_time
                
                self.log_data(elapsed, -speed, current_rpm, 0, 0, 0, current_pos)
                
                if int(elapsed) % 1 == 0 and elapsed - int(elapsed) < 0.1:
                    print(f"  시간: {elapsed:.1f}s, RPM: {current_rpm:.1f}, 위치: {current_pos}")
                
                time.sleep(0.1)
            
            self.motor.stop()
            self.stop_logging()
            
            # 결과 출력
            self.print_simple_graph('rpm')
            self.analyze_performance()
            
            time.sleep(1)
    
    def test_encoder_pulses(self, test_duration=10):
        """엔코더 펄스 확인 테스트"""
        print("=== 엔코더 펄스 확인 테스트 ===")
        print("모터를 수동으로 회전시켜보세요...")
        
        self.motor.reset_position()
        initial_pos = self.motor.get_position()
        
        self.start_logging()
        start_time = time.time()
        
        print("10초간 펄스 측정 중...")
        
        while time.time() - start_time < test_duration:
            current_pos = self.motor.get_position()
            elapsed = time.time() - start_time
            
            self.log_data(elapsed, 0, 0, 0, 0, 0, current_pos)
            
            if int(elapsed) % 2 == 0 and elapsed - int(elapsed) < 0.1:
                print(f"  시간: {elapsed:.1f}s, 펄스: {current_pos}")
            
            time.sleep(0.1)
        
        self.stop_logging()
        
        # 결과 분석
        final_pos = self.motor.get_position()
        total_pulses = final_pos - initial_pos
        
        print(f"\n=== 엔코더 분석 결과 ===")
        print(f"초기 펄스: {initial_pos}")
        print(f"최종 펄스: {final_pos}")
        print(f"총 펄스 변화: {total_pulses}")
        print(f"측정 시간: {test_duration}초")
        print(f"초당 펄스: {total_pulses / test_duration:.1f}")
        
        if total_pulses > 0:
            # 1920 PPR 기준으로 RPM 계산
            rpm = (total_pulses / test_duration / 1920) * 60
            print(f"계산된 RPM: {rpm:.1f}")
        
        # 위치 그래프 출력
        self.print_simple_graph('position')
    
    def test_position_control(self, target_positions=[50, 100, -50, 200, 0]):
        """위치 제어 테스트"""
        print("=== 위치 제어 테스트 ===")
        
        # 기본 PID 파라미터
        kp, ki, kd = 0.1, 0.1, 0.1
        self.motor.set_position_pid_parameters(kp=kp, ki=ki, kd=kd)
        print(f"PID 파라미터: Kp={kp}, Ki={ki}, Kd={kd}")
        
        for i, target_pos in enumerate(target_positions):
            print(f"\n--- 테스트 {i+1}: 목표 위치 {target_pos} ---")
            
            self.motor.reset_position()
            self.start_logging()
            start_time = time.time()
            
            # 위치 제어
            while time.time() - start_time < 8:  # 8초 타임아웃
                current_pos = self.motor.control_position_pid(target_pos)
                elapsed = time.time() - start_time
                
                self.log_data(elapsed, target_pos, 0, kp, ki, kd, current_pos)
                
                # 목표 도달 체크
                error = abs(target_pos - current_pos)
                if error <= 3:  # 허용 오차 3
                    print(f"목표 도달! 위치: {current_pos}, 오차: {error}, 시간: {elapsed:.1f}s")
                    break
                
                if int(elapsed) % 1 == 0 and elapsed - int(elapsed) < 0.1:
                    print(f"  시간: {elapsed:.1f}s, 목표: {target_pos}, 현재: {current_pos}, 오차: {error}")
                
                time.sleep(0.1)
            
            self.motor.stop()
            self.stop_logging()
            
            # 결과 출력
            self.print_simple_graph('position')
            
            # 최종 결과
            final_pos = self.motor.get_position()
            final_error = abs(target_pos - final_pos)
            print(f"최종 결과: 목표={target_pos}, 도달={final_pos}, 오차={final_error}")
            
            time.sleep(1)
    
    def interactive_pid_tuning(self):
        """대화형 PID 튜닝"""
        print("=== 대화형 PID 튜닝 ===")
        print("PID 파라미터를 조정하면서 실시간으로 성능을 확인하세요.")
        
        # 기본값
        kp, ki, kd = 0.3, 2.0, 0.1
        target_rpm = 50.0
        test_duration = 5
        
        while True:
            print(f"\n현재 설정:")
            print(f"Kp={kp}, Ki={ki}, Kd={kd}")
            print(f"목표 RPM: {target_rpm}")
            print(f"테스트 시간: {test_duration}초")
            
            print(f"\n옵션:")
            print("1. 테스트 실행")
            print("2. Kp 조정")
            print("3. Ki 조정") 
            print("4. Kd 조정")
            print("5. 목표 RPM 변경")
            print("6. 테스트 시간 변경")
            print("7. 성능 분석")
            print("8. 그래프 출력")
            print("0. 종료")
            
            try:
                choice = input("\n선택하세요 (0-8): ").strip()
                
                if choice == '0':
                    break
                elif choice == '1':
                    self.run_pid_test(kp, ki, kd, target_rpm, test_duration)
                elif choice == '2':
                    kp = float(input(f"새로운 Kp 값 (현재: {kp}): "))
                elif choice == '3':
                    ki = float(input(f"새로운 Ki 값 (현재: {ki}): "))
                elif choice == '4':
                    kd = float(input(f"새로운 Kd 값 (현재: {kd}): "))
                elif choice == '5':
                    target_rpm = float(input(f"새로운 목표 RPM (현재: {target_rpm}): "))
                elif choice == '6':
                    test_duration = int(input(f"새로운 테스트 시간 (현재: {test_duration}): "))
                elif choice == '7':
                    if self.data_log:
                        self.analyze_performance()
                    else:
                        print("분석할 데이터가 없습니다. 먼저 테스트를 실행하세요.")
                elif choice == '8':
                    if self.data_log:
                        self.print_simple_graph('rpm')
                    else:
                        print("출력할 데이터가 없습니다. 먼저 테스트를 실행하세요.")
                else:
                    print("잘못된 선택입니다.")
                    
            except ValueError:
                print("잘못된 입력입니다. 숫자를 입력해주세요.")
            except KeyboardInterrupt:
                print("\n튜닝을 중단합니다.")
                break
        
        self.motor.stop()
        print("대화형 PID 튜닝 종료")
    
    def run_pid_test(self, kp, ki, kd, target_rpm, test_duration):
        """PID 테스트 실행"""
        print(f"\n=== PID 테스트 실행 ===")
        print(f"파라미터: Kp={kp}, Ki={ki}, Kd={kd}")
        print(f"목표 RPM: {target_rpm}, 시간: {test_duration}초")
        
        # PID 파라미터 설정
        self.motor.set_speed_pid_parameters(kp=kp, ki=ki, kd=kd)
        
        # 테스트 실행
        self.motor.reset_position()
        self.start_logging()
        start_time = time.time()
        
        print("테스트 시작...")
        
        while time.time() - start_time < test_duration:
            current_rpm = self.motor.control_speed_pid(target_rpm)
            current_pos = self.motor.get_position()
            elapsed = time.time() - start_time
            
            self.log_data(elapsed, target_rpm, current_rpm, kp, ki, kd, current_pos)
            
            if int(elapsed) % 1 == 0 and elapsed - int(elapsed) < 0.1:
                print(f"  시간: {elapsed:.1f}s, 목표: {target_rpm}, 현재: {current_rpm:.1f} RPM")
            
            time.sleep(0.1)
        
        self.motor.stop()
        self.stop_logging()
        
        # 결과 분석
        self.analyze_performance()
        
        # 그래프 출력
        self.print_simple_graph('rpm')
    
    def auto_pid_tuning(self, target_rpm=50.0):
        """자동 PID 튜닝"""
        print("=== 자동 PID 튜닝 ===")
        print(f"목표 RPM: {target_rpm}")
        
        # 테스트할 파라미터 조합
        test_configs = [
            {"name": "보수적", "kp": 0.2, "ki": 1.0, "kd": 0.05},
            {"name": "균형", "kp": 0.3, "ki": 2.0, "kd": 0.1},
            {"name": "적극적", "kp": 0.4, "ki": 3.0, "kd": 0.15},
            {"name": "P+I만", "kp": 0.3, "ki": 2.5, "kd": 0.0},
            {"name": "P만", "kp": 0.5, "ki": 0.0, "kd": 0.0},
            {"name": "I만", "kp": 0.0, "ki": 3.0, "kd": 0.0}
        ]
        
        results = []
        
        for config in test_configs:
            print(f"\n--- {config['name']} 테스트 ---")
            print(f"파라미터: Kp={config['kp']}, Ki={config['ki']}, Kd={config['kd']}")
            
            # 테스트 실행
            self.run_pid_test(config['kp'], config['ki'], config['kd'], target_rpm, 5)
            
            # 결과 저장
            if self.data_log:
                analysis = self.analyze_performance()
                results.append({
                    'name': config['name'],
                    'params': config,
                    'performance': analysis
                })
            
            time.sleep(1)
        
        # 최적 파라미터 추천
        if results:
            print(f"\n=== 자동 튜닝 결과 ===")
            print(f"{'설정':<10} {'성공률':<8} {'표준편차':<10} {'평균오차':<8}")
            print("-" * 40)
            
            # 성공률 기준으로 정렬
            results.sort(key=lambda x: x['performance']['success_rate'], reverse=True)
            
            for result in results:
                perf = result['performance']
                print(f"{result['name']:<10} {perf['success_rate']:<8.1f}% {perf['std_dev']:<10.2f} {perf['avg_error']:<8.1f}")
            
            # 최적 설정 추천
            best_config = results[0]
            print(f"\n=== 최적 설정 추천 ===")
            print(f"설정: {best_config['name']}")
            print(f"파라미터: Kp={best_config['params']['kp']}, Ki={best_config['params']['ki']}, Kd={best_config['params']['kd']}")
            print(f"성공률: {best_config['performance']['success_rate']:.1f}%")
            print(f"표준편차: {best_config['performance']['std_dev']:.2f} RPM")
            
            return best_config['params']
        
        return None
    
    def main_menu(self):
        """메인 메뉴"""
        while True:
            print(f"\n{'='*50}")
            print("모터 개발자 도구 - 메인 메뉴")
            print(f"{'='*50}")
            print("1. 정역 RPM 제어 테스트")
            print("2. 엔코더 펄스 확인")
            print("3. 위치 제어 테스트")
            print("4. 대화형 PID 튜닝")
            print("5. 자동 PID 튜닝")
            print("6. 성능 분석")
            print("7. 그래프 출력")
            print("0. 종료")
            
            try:
                choice = input("\n선택하세요 (0-7): ").strip()
                
                if choice == '0':
                    print("프로그램을 종료합니다.")
                    break
                elif choice == '1':
                    self.test_forward_reverse_rpm()
                elif choice == '2':
                    self.test_encoder_pulses()
                elif choice == '3':
                    self.test_position_control()
                elif choice == '4':
                    self.interactive_pid_tuning()
                elif choice == '5':
                    target_rpm = float(input("목표 RPM을 입력하세요 (기본값: 50): ") or "50")
                    self.auto_pid_tuning(target_rpm)
                elif choice == '6':
                    if self.data_log:
                        self.analyze_performance()
                    else:
                        print("분석할 데이터가 없습니다. 먼저 테스트를 실행하세요.")
                elif choice == '7':
                    if self.data_log:
                        data_type = input("그래프 타입 (rpm/position, 기본값: rpm): ").strip() or "rpm"
                        self.print_simple_graph(data_type)
                    else:
                        print("출력할 데이터가 없습니다. 먼저 테스트를 실행하세요.")
                else:
                    print("잘못된 선택입니다.")
                    
            except ValueError:
                print("잘못된 입력입니다. 숫자를 입력해주세요.")
            except KeyboardInterrupt:
                print("\n프로그램을 종료합니다.")
                break
        
        self.motor.stop()


def main():
    """메인 함수"""
    print("모터 개발자 도구 시작...")
    
    tool = MotorDeveloperTool()
    
    try:
        tool.main_menu()
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        tool.motor.stop()
        print("모터 정지 완료")


if __name__ == "__main__":
    main()
